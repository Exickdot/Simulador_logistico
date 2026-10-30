"""
Unit Test Script for Plant Physics and Firmware Logic Verification
"""
import sys
sys.path.append('emulator')
from plant_emulator import PlantPhysicsModel, VirtualArduinoFirmware

def test_closed_loops():
    physics = PlantPhysicsModel(dt=0.05)
    firmware = VirtualArduinoFirmware()
    
    print("[TEST 1] Verificando Lazo 1: Rampa de Velocidad y Frenado...")
    # Test distance > 25 -> PWM = 200
    m_pwm, l_pwm, s_ang = firmware.execute_cycle(0.05, pv_dist=30.0, pv_lux=70.0, e_stop=False)
    assert firmware.fsm_state == "TRANSP"
    assert m_pwm == 200, f"Expected 200, got {m_pwm}"
    
    # Test distance in ramp: 15 cm -> factor = (15-11.5)/(20-11.5) = 3.5/8.5 -> PWM = int(80 + factor * 120) = 129
    m_pwm, l_pwm, s_ang = firmware.execute_cycle(0.05, pv_dist=15.0, pv_lux=70.0, e_stop=False)
    assert m_pwm == 129, f"Expected 129, got {m_pwm}"
    
    # Test distance <= 11.5 cm -> PWM = 0 (Stop en inspección)
    m_pwm, l_pwm, s_ang = firmware.execute_cycle(0.05, pv_dist=11.0, pv_lux=70.0, e_stop=False)
    assert m_pwm == 0, f"Expected 0, got {m_pwm}"
    print("  -> Lazo 1 PASÓ exitosamente.")

    print("[TEST 2] Verificando Lazo 3: Estabilización Lumínica...")
    # Ambient drop: PV = 40%, SP = 70% -> Error = +30% -> calc_pwm = 128 + (2.5 * 30 * 2.55) = 128 + 191.25 = 319 -> constrain 255
    m_pwm, l_pwm, s_ang = firmware.execute_cycle(0.05, pv_dist=20.0, pv_lux=40.0, e_stop=False)
    assert l_pwm == 255, f"Expected 255, got {l_pwm}"
    
    # High ambient: PV = 80%, SP = 70% -> Error = -10% -> calc_pwm = 128 - (2.5 * 10 * 2.55) = 128 - 63.75 = 64
    m_pwm, l_pwm, s_ang = firmware.execute_cycle(0.05, pv_dist=20.0, pv_lux=80.0, e_stop=False)
    assert l_pwm == 64, f"Expected 64, got {l_pwm}"
    print("  -> Lazo 3 PASÓ exitosamente.")

    print("[TEST 3] Verificando Parada de Emergencia (ISR)...")
    m_pwm, l_pwm, s_ang = firmware.execute_cycle(0.05, pv_dist=20.0, pv_lux=70.0, e_stop=True)
    assert firmware.fsm_state == "EMERG!"
    assert m_pwm == 0 and l_pwm == 0
    print("  -> Interrupción de Emergencia PASÓ exitosamente.")

    print("\nTODOS LOS TESTS DE CONTROL PASARON SATISFACTORIAMENTE (100% OK)")

if __name__ == "__main__":
    test_closed_loops()
