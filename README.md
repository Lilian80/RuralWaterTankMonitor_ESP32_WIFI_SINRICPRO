#### RuralWaterTankMonitor-ESP32 WIFI/SINRICPRO ####

# Sistema embarcado para medir o nível de água em reservatório Fortlev 1000 L usando ESP32 + HC-SR04, com indicação local por LEDs e telemetria/notificações via WIFI/Sinric Pro.
 
# Projetado para ambientes remotos (pastos e áreas externas), com arquitetura voltada à sustentabilidade: ciclos de leitura, deep sleep para baixo consumo e monitoramento de bateria para maximizar autonomia.

# Alimentação baseada em bateria 18650 (1S), com carga via TP4056 e step-up MT3608 para 5 V. O firmware envia nível/distância/bateria ao Sinric Pro e utiliza LEDs (Azul/Verde/Amarelo/Vermelho) como indicador visual do status do reservatório. 

## Como compilar
1. Copie `secrets_nivel.example.h` para `secrets_nivel.h`
2. Preencha as credenciais (Sinric e Wi-Fi)
3. Abra `RuralWaterTankMonitor-ESP32/SINRIC.ino` no Arduino IDE e envie para o ESP32


### Autor: Lilian de Ávila Costa ###
  # Versão: 1.2.0
  # Revisão: 17/12/2025