#ifndef WEB_PAGE_H
#define WEB_PAGE_H

#include <Arduino.h>

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Blue Caleche</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 0; background: #f4f4f9; color: #333; }
    .container {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      max-width: 1100px;
      margin: 20px auto;
      padding: 0 16px;
    }
    .box, .messages, .controls {
      background: white;
      border: 1px solid #ddd;
      border-radius: 8px;
      padding: 16px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.08);
    }
    .label { color: #666; font-size: 0.9rem; }
    .value { font-size: 1.6rem; font-weight: bold; margin-top: 6px; }
    .messages { grid-column: 1 / -1; min-height: 80px; white-space: pre-wrap; }
    .controls { grid-column: 1 / -1; }
    .button-row { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 16px; }
    button {
      border: 0;
      border-radius: 6px;
      padding: 10px 16px;
      background: #315efb;
      color: white;
      font-size: 1rem;
      cursor: pointer;
    }
    button:active { transform: translateY(1px); }
    .slider-row { display: grid; grid-template-columns: 1fr auto; gap: 12px; align-items: center; }
    input[type="range"] { width: 100%; }
    #status { grid-column: 1 / -1; color: #a00; }
    #status.connected { color: #087f23; }
    @media (max-width: 700px) {
      .container { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
    @media (max-width: 450px) {
      .container { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main class="container">
    <div id="status">Connecting...</div>
    <div class="box"><div class="label">RPM</div><div class="value" id="rpm">--</div></div>
    <div class="box"><div class="label">Speed (km/h)</div><div class="value" id="speed">--</div></div>
    <div class="box"><div class="label">Brake lever (0-1000)</div><div class="value" id="brakeLeverPosition">--</div></div>
    <div class="box"><div class="label">Brake joystick ADC raw</div><div class="value" id="brakeLeverRawValue">--</div></div>
    <div class="box"><div class="label">Front brake servo</div><div class="value" id="frontServoPosition">--</div></div>
    <div class="box"><div class="label">Back brake servo</div><div class="value" id="backServoPosition">--</div></div>
    <div class="box"><div class="label">Handbrake</div><div class="value" id="handBrakeEnabled">--</div></div>
    <div class="box"><div class="label">GPIO 10 handbrake LED</div><div class="value" id="handBrakeLedOn">--</div></div>
    <div class="box"><div class="label">Night lights</div><div class="value" id="nightLightsOn">--</div></div>
    <div class="box"><div class="label">Horn</div><div class="value" id="hornOn">--</div></div>
    <section class="controls">
      <div class="label">Simulated controls</div>
      <div class="button-row">
        <button type="button" onclick="sendCommand('nightButton')">Night-light button</button>
        <button type="button" onclick="sendCommand('hornButton')">Horn button</button>
        <button type="button" onclick="sendCommand('handBrakeButton')">Handbrake button</button>
        <button type="button" onclick="usePhysicalPot()">Use physical joystick</button>
      </div>
      <div class="slider-row">
        <input id="potSlider" type="range" min="0" max="1000" value="0" oninput="setSimulatedPot(this.value)">
        <strong id="potSliderValue">0</strong>
      </div>
    </section>
    <div class="messages" id="messages"></div>
  </main>
  <script>
    const statusElement = document.getElementById("status");
    let socket;
    let reconnectTimer;
    let simulatedPotEnabled = false;

    function sendCommand(command) {
      if (socket && socket.readyState === WebSocket.OPEN)
        socket.send(command);
    }

    function setSimulatedPot(value) {
      simulatedPotEnabled = true;
      document.getElementById("potSliderValue").textContent = value;
      sendCommand(`pot:${value}`);
    }

    function usePhysicalPot() {
      simulatedPotEnabled = false;
      sendCommand("potPhysical");
    }

    function connect() {
      socket = new WebSocket(`ws://${location.hostname}:81/`);

      socket.onopen = () => {
        statusElement.textContent = "Connected";
        statusElement.classList.add("connected");
      };

      socket.onclose = () => {
        statusElement.textContent = "Disconnected - reconnecting...";
        statusElement.classList.remove("connected");
        clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(connect, 1000);
      };

      socket.onerror = () => socket.close();

      socket.onmessage = event => {
        let data;
        try {
          data = JSON.parse(event.data);
        } catch (error) {
          return;
        }

        for (const field of ["rpm", "speed", "brakeLeverPosition", "brakeLeverRawValue",
                             "frontServoPosition", "backServoPosition"]) {
          if (data[field] !== undefined)
            document.getElementById(field).textContent = data[field];
        }

        if (data.handBrakeEnabled !== undefined)
        {
          document.getElementById("handBrakeEnabled").textContent =
            data.handBrakeEnabled ? "ON" : "OFF";
          document.getElementById("handBrakeLedOn").textContent =
            data.handBrakeEnabled ? "ON" : "OFF";
        }

        if (!simulatedPotEnabled && data.brakeLeverPosition !== undefined) {
          document.getElementById("potSlider").value = data.brakeLeverPosition;
          document.getElementById("potSliderValue").textContent = data.brakeLeverPosition;
        }

        if (data.nightLightsOn !== undefined)
          document.getElementById("nightLightsOn").textContent =
            data.nightLightsOn ? "ON" : "OFF";

        if (data.hornOn !== undefined)
          document.getElementById("hornOn").textContent =
            data.hornOn ? "ON" : "OFF";

        if (data.message !== undefined) {
          const messages = document.getElementById("messages");
          messages.textContent += (messages.textContent ? "\n" : "") + data.message;
        }
      };
    }

    connect();
  </script>
</body>
</html>
)rawliteral";

#endif
