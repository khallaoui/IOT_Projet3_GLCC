/**
 * ============================================================
 * Projet 3 – Detecteur de fuite de gaz et alarme incendie
 * Version : 1.0.0
 * Date    : 2026
 * GITHUB   : https://github.com/khallaoui/IOT_Projet3_GLCC
 * ============================================================
 */

// =================== BLYNK CONFIG ===================
#define BLYNK_TEMPLATE_ID   "TMPL2RxwoAkJD"
#define BLYNK_TEMPLATE_NAME "Detecteur Gaz Alarme"
#define BLYNK_AUTH_TOKEN    "sDdmmvzZRTelqk-jXRvsGGRZ2BgAL3SY"
#define BLYNK_PRINT         Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

// =================== WIFI ===================
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

// =================== PINS ===================
#define POT_GAZ      34
#define POT_FUMEE    35
#define DHT_PIN       4
#define BTN_ACK      13
#define LED_VERT     25
#define LED_JAUNE    26
#define LED_ROUGE    27
#define BUZZER       14

// =================== SEUILS ===================
#define SEUIL_NORMAL        1365      // ~33% de 4095
#define SEUIL_PREALERTE     2730      // ~66% de 4095
#define DELAI_PREALERTE_MS  30000UL   // 30s en pre-alerte -> alarme auto
#define INTERVALLE_BIP_MS   1000UL    // periode bip buzzer pre-alerte
#define INTERVALLE_BLYNK_MS 1000UL    // periode envoi Blynk
#define INTERVALLE_SERIAL_MS 300UL    // periode affichage Serial
#define DEBOUNCE_MS         50UL      // anti-rebond bouton
#define TIMEOUT_DHT_MS      2000UL    // timeout lecture DHT22

// =================== VIRTUAL PINS BLYNK ===================
#define V_GAZ        V0
#define V_FUMEE      V1
#define V_ACK_BTN    V2
#define V_ETAT       V3
#define V_TEMP       V4
#define V_HISTORIQUE V5
#define V_COMPTEUR   V6

// =================== ETATS ===================
/**
 * Enum representant les 3 niveaux d'alerte du systeme.
 * NORMAL     : pas de danger detecte
 * PRE_ALERTE : niveau intermediaire, surveillance requise
 * ALARME     : danger confirme, intervention necessaire
 */
enum Etat { NORMAL, PRE_ALERTE, ALARME };

// =================== VARIABLES GLOBALES ===================
DHT dht(DHT_PIN, DHT22);
BlynkTimer timer;
WidgetTerminal terminal(V_HISTORIQUE);

// Etats systeme
Etat etatGaz           = NORMAL;
Etat etatFumee         = NORMAL;
Etat etatGlobal        = NORMAL;
Etat dernierEtatGlobal = NORMAL;

// Flags
bool acquitte          = false;
bool etatBuzzer        = false;
bool enPreAlerte       = false;
bool wifiConnecte      = false;
bool btnPrecedent      = HIGH;

// Timers millis()
unsigned long lastBip          = 0;
unsigned long lastBlynk        = 0;
unsigned long lastSerial       = 0;
unsigned long lastBtn          = 0;
unsigned long debutPreAlerte   = 0;
unsigned long derniereLecture  = 0;

// Capteurs
int valGaz  = 0;
int valFum  = 0;
float temp  = 0.0;
float hum   = 0.0;

// Compteur alarmes
int compteurAlarmes = 0;

// ============================================================
//  FONCTIONS UTILITAIRES
// ============================================================

/**
 * calculerEtat()
 * Role    : Convertit une valeur ADC (0-4095) en niveau d'alerte.
 * Param   : val - valeur brute lue par analogRead()
 * Retour  : Etat (NORMAL, PRE_ALERTE ou ALARME)
 * Erreur  : si val hors plage [0-4095], clamp automatique
 */
Etat calculerEtat(int val) {
  if (val < 0)    val = 0;
  if (val > 4095) val = 4095;
  if (val >= SEUIL_PREALERTE) return ALARME;
  if (val >= SEUIL_NORMAL)    return PRE_ALERTE;
  return NORMAL;
}

/**
 * priorite()
 * Role    : Retourne l'etat le plus critique entre gaz et fumee.
 *           Priorite : ALARME > PRE_ALERTE > NORMAL
 * Param   : g - etat capteur gaz
 *           f - etat capteur fumee
 * Retour  : Etat le plus critique
 */
Etat priorite(Etat g, Etat f) {
  if (f == ALARME     || g == ALARME)     return ALARME;
  if (f == PRE_ALERTE || g == PRE_ALERTE) return PRE_ALERTE;
  return NORMAL;
}

/**
 * etatToString()
 * Role    : Convertit un Etat en chaine lisible.
 * Param   : e - etat a convertir
 * Retour  : String ("NORMAL", "PRE-ALERTE" ou "ALARME")
 */
String etatToString(Etat e) {
  switch (e) {
    case ALARME:     return "ALARME";
    case PRE_ALERTE: return "PRE-ALERTE";
    default:         return "NORMAL";
  }
}

/**
 * heureSimulee()
 * Role    : Genere une heure simulee basee sur millis().
 * Retour  : String au format "Xh Ym Zs"
 */
String heureSimulee() {
  unsigned long sec = millis() / 1000;
  return String(sec / 3600) + "h" +
         String((sec % 3600) / 60) + "m" +
         String(sec % 60) + "s";
}

// ============================================================
//  FONCTIONS LECTURE CAPTEURS
// ============================================================

/**
 * lireCapteurs()
 * Role    : Lit les 2 potentiometres et le DHT22.
 *           Utilise millis() pour ne pas bloquer.
 *           Gere le cas DHT non-reponse (isnan).
 * Retour  : void — met a jour les variables globales
 *           valGaz, valFum, temp, hum
 */
void lireCapteurs() {
  // Lecture ADC (toujours disponible, pas de timeout necessaire)
  int rawGaz = analogRead(POT_GAZ);
  int rawFum = analogRead(POT_FUMEE);

  // Validation plage ADC
  valGaz = constrain(rawGaz, 0, 4095);
  valFum = constrain(rawFum, 0, 4095);

  // Lecture DHT22 (lente, ~500ms) — uniquement si delai ecoule
  if (millis() - derniereLecture >= TIMEOUT_DHT_MS) {
    derniereLecture = millis();
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    // Gestion erreur capteur DHT non-reponse
    if (!isnan(t) && t > -40.0 && t < 80.0) {
      temp = t;
    } else {
      Serial.println("[WARN] DHT22 : lecture temperature invalide");
    }
    if (!isnan(h) && h >= 0.0 && h <= 100.0) {
      hum = h;
    } else {
      Serial.println("[WARN] DHT22 : lecture humidite invalide");
    }
  }
}

// ============================================================
//  FONCTIONS MACHINE A ETATS
// ============================================================

/**
 * miseAJourEtats()
 * Role    : Calcule les etats gaz, fumee et global
 *           a partir des valeurs ADC lues.
 *           Applique la priorite alarme > pre-alerte > normal.
 * Retour  : void — met a jour etatGaz, etatFumee, etatGlobal
 */
void miseAJourEtats() {
  etatGaz   = calculerEtat(valGaz);
  etatFumee = calculerEtat(valFum);
  etatGlobal = priorite(etatGaz, etatFumee);
}

/**
 * gererPreAlerteAuto()
 * Role    : Partie E — si le systeme reste en PRE_ALERTE
 *           plus de DELAI_PREALERTE_MS (30s), force ALARME.
 *           Utilise millis() sans aucun delay().
 * Retour  : void — peut modifier etatGlobal
 */
void gererPreAlerteAuto() {
  if (etatGlobal == PRE_ALERTE) {
    if (!enPreAlerte) {
      enPreAlerte    = true;
      debutPreAlerte = millis();
      Serial.println("[INFO] Debut minuterie pre-alerte (30s)");
    } else if (millis() - debutPreAlerte >= DELAI_PREALERTE_MS) {
      etatGlobal = ALARME;
      Serial.println("[AUTO] PRE-ALERTE > 30s => ALARME automatique");
      if (wifiConnecte) {
        terminal.println("[AUTO] Alarme apres 30s pre-alerte : " + heureSimulee());
        terminal.flush();
      }
    }
  } else {
    // Reset timer si on sort de pre-alerte
    if (enPreAlerte) {
      Serial.println("[INFO] Reset minuterie pre-alerte");
    }
    enPreAlerte    = false;
    debutPreAlerte = 0;
  }
}

// ============================================================
//  FONCTIONS ACTIONNEURS
// ============================================================

/**
 * gererLEDs()
 * Role    : Allume la LED correspondant a l'etat global.
 *           Une seule LED allumee a la fois.
 * Retour  : void
 */
void gererLEDs() {
  digitalWrite(LED_VERT,  etatGlobal == NORMAL      ? HIGH : LOW);
  digitalWrite(LED_JAUNE, etatGlobal == PRE_ALERTE  ? HIGH : LOW);
  digitalWrite(LED_ROUGE, etatGlobal == ALARME      ? HIGH : LOW);
}

/**
 * gererBuzzer()
 * Role    : Pilote le buzzer selon l'etat et l'acquittement.
 *           NORMAL     -> silence
 *           PRE_ALERTE -> bip 1/s (via millis, sans delay)
 *           ALARME     -> bip continu
 *           Acquitte   -> silence (LED rouge maintenue par gererLEDs)
 * Retour  : void
 */
void gererBuzzer() {
  if (acquitte) {
    digitalWrite(BUZZER, LOW);
    return;
  }
  switch (etatGlobal) {
    case NORMAL:
      digitalWrite(BUZZER, LOW);
      break;
    case PRE_ALERTE:
      if (millis() - lastBip >= INTERVALLE_BIP_MS) {
        lastBip    = millis();
        etatBuzzer = !etatBuzzer;
        digitalWrite(BUZZER, etatBuzzer ? HIGH : LOW);
      }
      break;
    case ALARME:
      digitalWrite(BUZZER, HIGH);
      break;
  }
}

// ============================================================
//  FONCTIONS BOUTON
// ============================================================

/**
 * gererBouton()
 * Role    : Detecte l'appui sur le bouton physique (GPIO13)
 *           avec anti-rebond via millis().
 *           Active l'acquittement local.
 * Retour  : void — peut modifier acquitte
 */
void gererBouton() {
  bool btnActuel = digitalRead(BTN_ACK);

  // Front descendant + debounce
  if (btnActuel == LOW && btnPrecedent == HIGH) {
    if (millis() - lastBtn >= DEBOUNCE_MS) {
      lastBtn  = millis();
      acquitte = true;
      Serial.println("[BTN] Alarme acquittee via bouton physique");
      if (wifiConnecte) {
        terminal.println("[BTN] ACQ physique : " + heureSimulee());
        terminal.flush();
      }
    }
  }
  btnPrecedent = btnActuel;

  // Reset auto acquittement si retour normal
  if (etatGlobal == NORMAL && acquitte) {
    acquitte = false;
    Serial.println("[INFO] Retour NORMAL : reset acquittement");
    if (wifiConnecte) {
      terminal.println("[OK] Retour NORMAL : " + heureSimulee());
      terminal.flush();
    }
  }
}

// ============================================================
//  FONCTIONS BLYNK
// ============================================================

/**
 * BLYNK_WRITE(V_ACK_BTN)
 * Role    : Callback Blynk — recu quand le bouton V2
 *           est presse dans l'app Blynk (acquittement distant).
 * Param   : param.asInt() == 1 -> acquittement active
 */
BLYNK_WRITE(V_ACK_BTN) {
  if (param.asInt() == 1) {
    acquitte = true;
    Serial.println("[BLYNK] Alarme acquittee a distance");
    terminal.println("[BLYNK] ACQ distant : " + heureSimulee());
    terminal.flush();
  }
}

/**
 * BLYNK_CONNECTED()
 * Role    : Callback appele a chaque reconnexion Blynk.
 *           Synchronise l'etat du bouton V2.
 */
BLYNK_CONNECTED() {
  wifiConnecte = true;
  Blynk.syncVirtual(V_ACK_BTN);
  Serial.println("[BLYNK] Connecte et synchronise");
}

/**
 * notifierAlarme()
 * Role    : Envoie une notification push Blynk,
 *           log dans le terminal et incremente le compteur.
 * Param   : type - chaine decrivant le type d'alarme
 * Retour  : void
 */
void notifierAlarme(String type) {
  compteurAlarmes++;
  String msg = "[" + heureSimulee() + "] ALARME-" + type +
               " (total:" + String(compteurAlarmes) + ")";
  Serial.println(">>> " + msg);

  if (!wifiConnecte) return;

  terminal.println(msg);
  terminal.flush();
  Blynk.logEvent("alarme", type + " detecte");
}

/**
 * gererNotifications()
 * Role    : Detecte les transitions vers ALARME et
 *           appelle notifierAlarme() une seule fois par transition.
 * Retour  : void
 */
void gererNotifications() {
  if (etatGlobal == ALARME && dernierEtatGlobal != ALARME) {
    String type = "GAZ+FUMEE";
    if      (etatFumee == ALARME && etatGaz != ALARME) type = "FUMEE";
    else if (etatGaz   == ALARME && etatFumee != ALARME) type = "GAZ";
    notifierAlarme(type);
  }
  dernierEtatGlobal = etatGlobal;
}

/**
 * envoyerBlynk()
 * Role    : Envoie toutes les donnees vers Blynk.
 *           Appelee par BlynkTimer toutes les 1s.
 *           Verifie la connexion avant chaque envoi.
 * Retour  : void
 */
void envoyerBlynk() {
  if (!Blynk.connected()) {
    wifiConnecte = false;
    Serial.println("[WARN] Blynk deconnecte — tentative reconnexion...");
    Blynk.connect(3000); // timeout 3s
    if (!Blynk.connected()) {
      Serial.println("[ERR] Blynk : reconnexion echouee");
      return;
    }
    wifiConnecte = true;
    Serial.println("[INFO] Blynk : reconnecte");
  }

  int pctGaz = map(valGaz, 0, 4095, 0, 100);
  int pctFum = map(valFum, 0, 4095, 0, 100);
  int etatInt = (etatGlobal == NORMAL) ? 0 :
                (etatGlobal == PRE_ALERTE) ? 1 : 2;

  Blynk.virtualWrite(V_GAZ,      pctGaz);
  Blynk.virtualWrite(V_FUMEE,    pctFum);
  Blynk.virtualWrite(V_ETAT,     etatInt);
  Blynk.virtualWrite(V_COMPTEUR, compteurAlarmes);
  if (temp != 0.0) Blynk.virtualWrite(V_TEMP, temp);
}

// ============================================================
//  FONCTION AFFICHAGE SERIAL
// ============================================================

/**
 * afficherSerial()
 * Role    : Affiche l'etat complet du systeme dans le
 *           Serial Monitor. Appelee via millis() toutes
 *           les INTERVALLE_SERIAL_MS ms.
 * Retour  : void
 */
void afficherSerial() {
  if (millis() - lastSerial < INTERVALLE_SERIAL_MS) return;
  lastSerial = millis();

  Serial.print("GAZ:");    Serial.print(map(valGaz, 0, 4095, 0, 100));
  Serial.print("% | FUM:"); Serial.print(map(valFum, 0, 4095, 0, 100));
  Serial.print("% | TEMP:");
  Serial.print(temp, 1);   Serial.print("C");
  Serial.print(" | HUM:");  Serial.print(hum, 1); Serial.print("%");
  Serial.print(" | ETAT:"); Serial.print(etatToString(etatGlobal));
  Serial.print(" | ACK:");  Serial.print(acquitte ? "OUI" : "NON");
  Serial.print(" | WiFi:"); Serial.print(wifiConnecte ? "OK" : "OFF");
  Serial.print(" | ALM:");  Serial.println(compteurAlarmes);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== Systeme Securite v1.0 ===");

  // Sorties
  pinMode(LED_VERT,  OUTPUT);
  pinMode(LED_JAUNE, OUTPUT);
  pinMode(LED_ROUGE, OUTPUT);
  pinMode(BUZZER,    OUTPUT);

  // Entrees
  pinMode(BTN_ACK, INPUT_PULLUP);

  // Etat initial : LED verte allumee
  digitalWrite(LED_VERT, HIGH);

  // DHT22
  dht.begin();
  Serial.println("[INFO] DHT22 initialise");

  // Connexion WiFi + Blynk
  Serial.println("[INFO] Connexion Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  wifiConnecte = Blynk.connected();
  if (wifiConnecte) {
    Serial.println("[INFO] Blynk connecte");
  } else {
    Serial.println("[WARN] Blynk non connecte — mode local actif");
  }

  // Timer Blynk
  timer.setInterval(INTERVALLE_BLYNK_MS, envoyerBlynk);

  Serial.println("=== Pret ===");
}

// ============================================================
//  LOOP — uniquement des appels de fonctions
// ============================================================
void loop() {
  // 1. Blynk
  Blynk.run();
  timer.run();

  // 2. Lecture capteurs
  lireCapteurs();

  // 3. Machine a etats
  miseAJourEtats();

  // 4. Escalade pre-alerte auto (Partie E)
  gererPreAlerteAuto();

  // 5. Bouton physique
  gererBouton();

  // 6. Notifications Blynk
  gererNotifications();

  // 7. Actionneurs
  gererLEDs();
  gererBuzzer();

  // 8. Affichage Serial
  afficherSerial();
}
