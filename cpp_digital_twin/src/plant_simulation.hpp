// =====================================================================================
// PROYECTO: ESTACIÓN LOGÍSTICA DE SELECCIÓN, CLASIFICACIÓN Y ENRUTAMIENTO INDUSTRIAL
// GEMELO DIGITAL NATIVO C++ (OPENGL / WIN32) - MODELO FÍSICO Y CONTROL
// =====================================================================================

#ifndef PLANT_SIMULATION_HPP
#define PLANT_SIMULATION_HPP

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdlib>

// Estados canónicos de la FSM
enum FsmState {
    FSM_INIT = 0,
    FSM_REPOSO = 1,
    FSM_TRANSP = 2,
    FSM_INSPEC = 3,
    FSM_DESVIO = 4,
    FSM_EMERG = 5
};

// Calidad de pieza
enum QualityType {
    QUAL_NINGUNA = 0,
    QUAL_CONFORME = 1,
    QUAL_RETRABAJO = 2,
    QUAL_RECHAZO = 3
};

// Estados cinemáticos de la caja
enum BoxMotionState {
    BOX_CAYENDO_DE_TOLVA = 0, // Cayendo por gravedad desde la tolva de alimentación
    BOX_EN_FAJA = 1,          // Desplazamiento sobre la faja
    BOX_EN_DESVIO = 2,        // Siendo guiada por la compuerta deflectora SG90
    BOX_EN_TOLVA = 3,         // Deslizándose por su rampa de destino
    BOX_FINALIZADA = 4        // Reposando en su gaveta receptora correspondiente
};

inline const char* GetFsmStateName(FsmState st) {
    switch (st) {
        case FSM_INIT:   return "INIT";
        case FSM_REPOSO: return "REPOSO";
        case FSM_TRANSP: return "TRANSP";
        case FSM_INSPEC: return "INSPEC";
        case FSM_DESVIO: return "DESVIO";
        case FSM_EMERG:  return "EMERG!";
        default:         return "DESCON";
    }
}

// Controlador PI discreto con Anti-Windup
class PIDController {
public:
    float Kp, Ki, Kd;
    float OutMin, OutMax;
    float Integral;
    float LastError;
    bool FirstRun;

    PIDController(float kp = 2.5f, float ki = 0.4f, float kd = 0.05f, float minOut = 0.0f, float maxOut = 255.0f)
        : Kp(kp), Ki(ki), Kd(kd), OutMin(minOut), OutMax(maxOut), Integral(0.0f), LastError(0.0f), FirstRun(true) {}

    void Reset() {
        Integral = 0.0f;
        LastError = 0.0f;
        FirstRun = true;
    }

    float Compute(float sp, float pv, float dt) {
        if (dt <= 0.0f) dt = 0.05f;
        float error = sp - pv;
        if (std::abs(error) < 0.5f) error = 0.0f; // Zona muerta

        float pTerm = Kp * error;
        Integral += Ki * error * dt;
        Integral = std::clamp(Integral, OutMin, OutMax);

        float dTerm = 0.0f;
        if (!FirstRun) {
            dTerm = Kd * (error - LastError) / dt;
        } else {
            FirstRun = false;
        }
        LastError = error;

        float output = pTerm + Integral + dTerm;
        return std::clamp(output, OutMin, OutMax);
    }
};

// Caja dinámica con caída de tolva de alimentación y clasificación en carro móvil indexador
struct PhysicalBox {
    QualityType Quality;
    float PosX, PosY, PosZ;
    float SizeX, SizeY, SizeZ;
    float RotY, RotZ;
    BoxMotionState State;
    int TargetChute; // 1: Reproceso, 2: Conforme, 3: Rechazo
    float ContainerOffsetZ; // Offset local en el carro móvil (-0.16, 0.0, +0.16)
    float SettleTimer;
    float ColorR, ColorG, ColorB;

    PhysicalBox(QualityType q = QUAL_CONFORME)
        : Quality(q), PosX(-0.46f), PosY(0.50f), PosZ(0.0f),
          SizeX(0.07f), SizeY(0.05f), SizeZ(0.07f),
          RotY(0.0f), RotZ(0.0f), State(BOX_CAYENDO_DE_TOLVA), TargetChute(2),
          ContainerOffsetZ(0.0f), SettleTimer(0.0f) {
        if (Quality == QUAL_RETRABAJO) {
            TargetChute = 1;
            ContainerOffsetZ = -0.16f; // Contenedor Ámbar montado a -0.16m en el carro
        } else if (Quality == QUAL_RECHAZO) {
            TargetChute = 3;
            ContainerOffsetZ = 0.16f;  // Contenedor Rojo montado a +0.16m en el carro
        } else {
            TargetChute = 2;
            ContainerOffsetZ = 0.0f;   // Contenedor Verde montado en el centro del carro
        }
        UpdateColors();
    }

    void UpdateColors() {
        if (Quality == QUAL_CONFORME) {
            ColorR = 0.28f; ColorG = 0.72f; ColorB = 0.32f; // Verde ingeniería
        } else if (Quality == QUAL_RETRABAJO) {
            ColorR = 0.95f; ColorG = 0.70f; ColorB = 0.15f; // Ámbar técnico
        } else {
            ColorR = 0.85f; ColorG = 0.22f; ColorB = 0.20f; // Rojo descarte
        }
    }

    void Update(float dt, float beltSpeed, float cartZ) {
        switch (State) {
            case BOX_CAYENDO_DE_TOLVA:
                // Caída libre por gravedad desde la tolva de alimentación sobre la faja
                PosY -= 0.65f * dt;
                if (PosY <= 0.425f) {
                    PosY = 0.425f;
                    State = BOX_EN_FAJA;
                }
                break;

            case BOX_EN_FAJA:
                // Desplazamiento sobre la faja hacia adelante
                PosX += beltSpeed * dt;
                PosY = 0.425f;
                PosZ = 0.0f;
                RotY = 0.0f;
                RotZ = 0.0f;

                // Al llegar a la cabeza de la faja (X >= 0.49)
                if (PosX >= 0.49f) {
                    State = BOX_EN_TOLVA;
                }
                break;

            case BOX_EN_DESVIO:
            case BOX_EN_TOLVA:
                // Deslizamiento fluido por el labio de descarga directo al contenedor que está alineado a Z = 0
                PosX += 0.40f * dt;
                PosY -= 0.65f * dt;
                RotZ = 20.0f;
                PosZ = 0.0f; // Cae en el centro donde está alineado el contenedor

                if (PosY <= 0.18f) {
                    PosY = 0.18f;
                    PosX = 0.62f;
                    RotZ = 0.0f;
                    State = BOX_FINALIZADA;
                }
                break;

            case BOX_FINALIZADA:
                // Reposa dentro del contenedor y se mueve de forma solidaria con el carro móvil
                PosX = 0.62f;
                PosY = 0.18f;
                PosZ = cartZ + ContainerOffsetZ;
                RotY = 0.0f;
                RotZ = 0.0f;
                SettleTimer += dt;
                break;
        }
    }
};

// Modelo integrado de física de planta y firmware virtual
class PlantSimulation {
public:
    FsmState State;
    bool EmergencyStop;

    // Lazo 1
    float UltrasonicDistCm;
    float BeltSpeedMps;
    int MotorPwm;

    // Lazo 2: Carro Móvil Indexador de Contenedores (M-102)
    float CartZ;
    float TargetCartZ;
    float CartSpeedMps;
    float ServoAngleDeg;
    QualityType LastQuality;

    // Lazo 3
    float AmbientLuxPct;
    float InspectionLuxPct;
    float TargetLuxPct;
    int LightPwm;
    PIDController LightPid;

    // Contadores
    int TotalProcessed;
    int CountConforme;
    int CountRetrabajo;
    int CountRechazo;

    // Cajas
    std::vector<PhysicalBox> Boxes;
    float BoxSpawnTimer;
    float InspectTimer;
    float DivertTimer;

    PlantSimulation()
        : State(FSM_REPOSO), EmergencyStop(false),
          UltrasonicDistCm(50.0f), BeltSpeedMps(0.0f), MotorPwm(0),
          CartZ(0.0f), TargetCartZ(0.0f), CartSpeedMps(0.0f), ServoAngleDeg(90.0f), LastQuality(QUAL_NINGUNA),
          AmbientLuxPct(40.0f), InspectionLuxPct(40.0f), TargetLuxPct(70.0f), LightPwm(25),
          TotalProcessed(0), CountConforme(0), CountRetrabajo(0), CountRechazo(0),
          BoxSpawnTimer(0.8f), InspectTimer(0.0f), DivertTimer(0.0f) {}

    void SpawnBox(QualityType q = QUAL_NINGUNA) {
        if (q == QUAL_NINGUNA) {
            int r = rand() % 100;
            if (r < 55) q = QUAL_CONFORME;
            else if (r < 80) q = QUAL_RETRABAJO;
            else q = QUAL_RECHAZO;
        }
        Boxes.emplace_back(q);
        if (State == FSM_REPOSO) {
            State = FSM_TRANSP;
        }
    }

    void TriggerEmergency(bool activate) {
        EmergencyStop = activate;
        if (activate) {
            State = FSM_EMERG;
            MotorPwm = 0;
            BeltSpeedMps = 0.0f;
            LightPwm = 0;
        }
    }

    void Update(float dt) {
        dt = std::min(0.05f, dt);

        // 1. Detección de distancia ultrasónica (sensor HC-SR04 en X = 0.35, mirando hacia -X)
        float closestDist = 50.0f;
        PhysicalBox* boxUnderCam = nullptr;

        for (auto& b : Boxes) {
            if (b.State == BOX_EN_FAJA) {
                float d = (0.35f - b.PosX) * 100.0f;
                if (d > 0.0f && d < closestDist) {
                    closestDist = d;
                }
                // Zona de detección óptica / cámara (X entre 0.18 y 0.30)
                if (b.PosX >= 0.18f && b.PosX <= 0.30f) {
                    boxUnderCam = &b;
                }
            }
        }
        UltrasonicDistCm = closestDist;

        // 2. Máquina de Estados Finitos (FSM)
        if (EmergencyStop) {
            State = FSM_EMERG;
            MotorPwm = 0;
            LightPwm = 0;
        } else {
            switch (State) {
                case FSM_REPOSO:
                    ServoAngleDeg = 90.0f;
                    MotorPwm = 0;
                    LightPwm = 25;
                    break;

                case FSM_TRANSP:
                    // Lazo 1: Rampa de velocidad y anticolisión
                    if (UltrasonicDistCm <= 5.0f) {
                        MotorPwm = 0;
                    } else if (UltrasonicDistCm >= 25.0f) {
                        MotorPwm = 200;
                    } else {
                        float f = (UltrasonicDistCm - 5.0f) / (25.0f - 5.0f);
                        MotorPwm = (int)(f * 200.0f);
                    }

                    // Transición a inspección
                    if (UltrasonicDistCm > 5.0f && UltrasonicDistCm <= 10.0f) {
                        State = FSM_INSPEC;
                        InspectTimer = 0.0f;
                    }
                    break;

                case FSM_INSPEC:
                    MotorPwm = 0; // Detención momentánea para inspección nítida
                    InspectTimer += dt;
                    if (InspectTimer >= 1.4f) {
                        if (boxUnderCam) {
                            LastQuality = boxUnderCam->Quality;
                        } else {
                            LastQuality = (InspectionLuxPct >= 65.0f) ? QUAL_CONFORME :
                                          ((InspectionLuxPct >= 40.0f) ? QUAL_RETRABAJO : QUAL_RECHAZO);
                        }

                        // Lazo 2: Posicionamiento del Carro Móvil Indexador (M-102)
                        if (LastQuality == QUAL_RETRABAJO) {
                            TargetCartZ = 0.16f; // Contenedor Ámbar (offset local -0.16) se alinea a Z = 0
                            ServoAngleDeg = 45.0f;
                        } else if (LastQuality == QUAL_RECHAZO) {
                            TargetCartZ = -0.16f; // Contenedor Rojo (offset local +0.16) se alinea a Z = 0
                            ServoAngleDeg = 135.0f;
                        } else {
                            TargetCartZ = 0.0f;  // Contenedor Verde (offset local 0.0) se alinea a Z = 0
                            ServoAngleDeg = 90.0f;
                        }

                        State = FSM_DESVIO;
                        DivertTimer = 0.0f;
                    }
                    break;

                case FSM_DESVIO:
                    MotorPwm = 175; // Avance constante hacia el carro
                    DivertTimer += dt;
                    if (DivertTimer >= 2.2f) {
                        State = FSM_TRANSP;
                    }
                    break;

                case FSM_INIT:
                default:
                    State = FSM_REPOSO;
                    break;
            }

            // Lazo 3: Estabilización de Iluminación con PI y Anti-Windup
            if (State == FSM_TRANSP || State == FSM_INSPEC) {
                LightPwm = (int)LightPid.Compute(TargetLuxPct, InspectionLuxPct, dt);
            } else {
                LightPwm = 25;
            }
        }

        // 3. Dinámica física de planta continua
        float targetSpeed = (MotorPwm / 255.0f) * 0.28f;
        float alphaM = std::exp(-dt / 0.14f);
        BeltSpeedMps = alphaM * BeltSpeedMps + (1.0f - alphaM) * targetSpeed;

        float targetLux = std::clamp(AmbientLuxPct + (LightPwm / 255.0f) * 60.0f, 0.0f, 100.0f);
        float alphaL = std::exp(-dt / 0.07f);
        InspectionLuxPct = alphaL * InspectionLuxPct + (1.0f - alphaL) * targetLux;

        // Dinámica del Carro Móvil Indexador M-102
        float dz = TargetCartZ - CartZ;
        float maxStep = 0.38f * dt; // Velocidad de traslación del carro: 38 cm/s
        if (std::abs(dz) <= maxStep) {
            CartZ = TargetCartZ;
            CartSpeedMps = 0.0f;
        } else {
            float dir = (dz > 0.0f) ? 1.0f : -1.0f;
            CartZ += dir * maxStep;
            CartSpeedMps = dir * 0.38f;
        }

        // 4. Actualización cinemática de cajas
        for (size_t i = 0; i < Boxes.size();) {
            Boxes[i].Update(dt, BeltSpeedMps, CartZ);

            // Contabilizar al finalizar y limpiar de memoria tras reposar
            if (Boxes[i].State == BOX_FINALIZADA && Boxes[i].SettleTimer >= 3.0f) {
                TotalProcessed++;
                if (Boxes[i].TargetChute == 1) CountRetrabajo++;
                else if (Boxes[i].TargetChute == 2) CountConforme++;
                else if (Boxes[i].TargetChute == 3) CountRechazo++;

                Boxes.erase(Boxes.begin() + i);
            } else {
                i++;
            }
        }

        // Generador automático de cajas desde la tolva de alimentación
        if (State != FSM_REPOSO && State != FSM_EMERG) {
            BoxSpawnTimer -= dt;
            if (BoxSpawnTimer <= 0.0f) {
                bool spaceAvailable = true;
                for (const auto& b : Boxes) {
                    if ((b.State == BOX_EN_FAJA || b.State == BOX_CAYENDO_DE_TOLVA) && b.PosX < -0.28f) {
                        spaceAvailable = false;
                        break;
                    }
                }
                if (spaceAvailable) {
                    SpawnBox();
                }
                BoxSpawnTimer = 3.6f + (float)(rand() % 15) * 0.1f;
            }
        }
    }
};

#endif // PLANT_SIMULATION_HPP
