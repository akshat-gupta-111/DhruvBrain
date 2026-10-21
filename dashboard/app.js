/**
 * ============================================================================
 * UNO R4 H-DRIVE BLE TELEMETRY & CONTROLLER
 * Web Bluetooth Client for Holonomic 4-Wheel Mecanum Drive
 * Supporting Unified Jetson Payload Format (<DIR,SPEED,TIME>|<LED,MOOD>),
 * Instant Zero-Buffer Mode Arbitration (Autonomous vs Manual),
 * Automatic Control Locking, and 100% Mirroring of Serial Monitor Telemetry
 * ============================================================================
 */

// Bluetooth UUID Configurations (Dual-support for Dhruv Cable Link & Legacy)
const BLE_CONFIG = {
  // Primary: Dhruv Jetson Cable Peripheral (H-drive-jetson-cable.ino)
  PRIMARY_SERVICE: '19b10000-e8f2-537e-4f6c-d104768a1214',
  PRIMARY_CHAR:    '19b10001-e8f2-537e-4f6c-d104768a1214',

  // Fallback: Legacy H-Drive Service
  LEGACY_SERVICE:  '12345678-1234-1234-1234-1234567890ab',
  LEGACY_CMD_CHAR: '12345678-1234-1234-1234-1234567890cd',
  LEGACY_LOG_CHAR: '12345678-1234-1234-1234-1234567890ef',

  DEVICE_PREFIX_1: 'Dhruv',
  DEVICE_PREFIX_2: 'UNO_R4'
};

// Jetson Parsed Command Mapping (<MOTOR_DIR,SPEED,TIME_MS>|<LED,MOOD>)
// Duration 0 runs continuously while button is held down; release sends <STOP>.
const JETSON_COMMAND_MAP = {
  'N':    '<FWD,180,0>|<LED,IDLE_WHITE>',
  'S':    '<REV,180,0>|<LED,IDLE_WHITE>',
  'W':    '<STRAFE_L,180,0>|<LED,IDLE_WHITE>',
  'E':    '<STRAFE_R,180,0>|<LED,IDLE_WHITE>',
  'NW':   '<DIAG_FL,180,0>|<LED,IDLE_WHITE>',
  'NE':   '<DIAG_FR,180,0>|<LED,IDLE_WHITE>',
  'SW':   '<DIAG_BL,180,0>|<LED,IDLE_WHITE>',
  'SE':   '<DIAG_BR,180,0>|<LED,IDLE_WHITE>',
  'CW':   '<PIVOT_R,180,0>|<LED,IDLE_WHITE>',
  'CCW':  '<PIVOT_L,180,0>|<LED,IDLE_WHITE>',
  'Q':    '<STOP>|<LED,IDLE_WHITE>',
  'X':    '<STOP>|<LED,IDLE_WHITE>',
  'STOP': '<STOP>|<LED,IDLE_WHITE>'
};

// Application State
const state = {
  bluetoothDevice: null,
  gattServer: null,
  cmdCharacteristic: null,
  logCharacteristic: null,
  isConnected: false,
  isConnecting: false,
  isAutonomous: true,          // Default Arduino state (Jetson Cable controls active)
  useJetsonFormat: true,       // Default: true, passes <FWD,180,0>|<LED,IDLE_WHITE>
  holdToMove: true,
  currentCmd: 'Q',
  activeSector: 'IDLE',
  txCount: 0,
  rxCount: 0,
  autoScroll: true,
  currentFilter: 'all',
  activeKeys: new Set(),
  joystickActive: false,
  lastSentCmd: null,
  lastSendTime: 0
};

// DOM Elements Cache
const dom = {
  btnConnectBle: document.getElementById('btnConnectBle'),
  btnConnectText: document.getElementById('btnConnectText'),
  connectionStatusPill: document.getElementById('connectionStatusPill'),
  statusDot: document.getElementById('statusDot'),
  statusLabel: document.getElementById('statusLabel'),
  btnToggleMode: document.getElementById('btnToggleMode'),
  btnModeText: document.getElementById('btnModeText'),
  chkJetsonFormat: document.getElementById('chkJetsonFormat'),
  panelControls: document.getElementById('panelControls'),
  panelJoystick: document.getElementById('panelJoystick'),
  matrixLockPill: document.getElementById('matrixLockPill'),
  joystickLockPill: document.getElementById('joystickLockPill'),
  statDevName: document.getElementById('statDevName'),
  statTxCount: document.getElementById('statTxCount'),
  statRxCount: document.getElementById('statRxCount'),
  bleWarningBanner: document.getElementById('bleWarningBanner'),
  chkHoldToMove: document.getElementById('chkHoldToMove'),
  matrixButtons: document.querySelectorAll('.cmd-btn'),
  rotationButtons: document.querySelectorAll('.cmd-rotation'),
  joystickBase: document.getElementById('joystickBase'),
  joystickStick: document.getElementById('joystickStick'),
  joystickActiveBadge: document.getElementById('joystickActiveBadge'),
  teleCmd: document.getElementById('teleCmd'),
  teleAngle: document.getElementById('teleAngle'),
  teleIntensity: document.getElementById('teleIntensity'),
  terminalContainer: document.getElementById('terminalContainer'),
  terminalBody: document.getElementById('terminalBody'),
  terminalInputForm: document.getElementById('terminalInputForm'),
  manualCmdInput: document.getElementById('manualCmdInput'),
  btnClearLog: document.getElementById('btnClearLog'),
  btnExportLog: document.getElementById('btnExportLog'),
  btnAutoScroll: document.getElementById('btnAutoScroll'),
  filterTabs: document.querySelectorAll('.tab-btn'),
  presetButtons: document.querySelectorAll('.preset-btn')
};

// ============================================================================
// INITIALIZATION
// ============================================================================

document.addEventListener('DOMContentLoaded', () => {
  initBleSupportCheck();
  initModeControls();
  initMatrixControls();
  initJoystickEngine();
  initKeyboardControls();
  initTerminalLogger();
  initPresetButtons();

  // Apply initial Autonomous Lock state (manual controls locked, terminal & mode active)
  updateModeUI(true);

  addLog('sys', 'READY', 'System ready. Click "Connect UNO R4" to establish Web Bluetooth link.');
  addLog('sys', 'CFG', 'Jetson Payload Format ACTIVE: North sends <FWD,180,0>|<LED,IDLE_WHITE>.');
  addLog('sys', 'MODE', 'Initial Mode: AUTONOMOUS. Manual dashboard controls locked until switched to MANUAL.');
});

// Check if Web Bluetooth API is supported
function initBleSupportCheck() {
  if (!navigator.bluetooth) {
    if (dom.bleWarningBanner) dom.bleWarningBanner.classList.remove('hidden');
    if (dom.btnConnectBle) {
      dom.btnConnectBle.disabled = true;
      dom.btnConnectBle.title = 'Web Bluetooth is not supported in this browser.';
    }
    addLog('err', 'WARN', 'Web Bluetooth API is not supported in this browser. Use Chrome/Edge with HTTPS or localhost.');
  }
}

// ============================================================================
// INSTANT ARBITRATION MODE SWITCHING & CONTROL LOCKING
// ============================================================================

function initModeControls() {
  if (dom.btnToggleMode) {
    dom.btnToggleMode.addEventListener('click', () => {
      // 1. Instant optimistic UI switch with zero delay
      const targetIsAuto = !state.isAutonomous;
      updateModeUI(targetIsAuto);

      // 2. Clear any active keys, joystick states, and deduplication buffers
      state.activeKeys.clear();
      state.lastSentCmd = null;

      addLog('sys', 'MODE', `Mode switch triggered ➔ ${targetIsAuto ? 'AUTONOMOUS (Jetson Brain Driving)' : 'MANUAL (Dashboard Active)'}`);

      // 3. Immediately transmit mode switch command over BLE to Arduino
      if (state.isConnected) {
        sendCommand(targetIsAuto ? 'MODE:AUTO' : 'MODE:MANUAL', true);
      } else {
        addLog('sys', 'INFO', 'Arduino will adopt this mode once Bluetooth connects.');
      }
    });
  }

  if (dom.chkJetsonFormat) {
    dom.chkJetsonFormat.addEventListener('change', (e) => {
      state.useJetsonFormat = e.target.checked;
      addLog('sys', 'CFG', `Payload Format: ${state.useJetsonFormat ? 'JETSON PARSED (<DIR,SPEED,TIME>|<LED,MOOD>)' : 'LEGACY TOKENS (N, S, W, E)'}`);
    });
  }
}

/**
 * Updates Mode UI and instantly locks or unlocks manual driving controls.
 * Terminal viewer and mode toggle ALWAYS remain active.
 */
function updateModeUI(isAuto) {
  state.isAutonomous = isAuto;

  // 1. Update Mode Header Button
  if (dom.btnToggleMode && dom.btnModeText) {
    if (isAuto) {
      dom.btnToggleMode.classList.remove('mode-manual');
      dom.btnToggleMode.classList.add('mode-auto');
      dom.btnModeText.textContent = 'AUTONOMOUS';
      dom.btnToggleMode.title = 'Mode: AUTONOMOUS. Jetson controls motors. Click to switch to MANUAL.';
    } else {
      dom.btnToggleMode.classList.remove('mode-auto');
      dom.btnToggleMode.classList.add('mode-manual');
      dom.btnModeText.textContent = 'MANUAL (ACTIVE)';
      dom.btnToggleMode.title = 'Mode: MANUAL. Dashboard controls motors. Click to switch to AUTO.';
    }
  }

  // 2. Instantly Lock / Unlock Manual Movement Controls
  const controlsLocked = isAuto;

  if (dom.panelControls) {
    dom.panelControls.classList.toggle('controls-locked', controlsLocked);
  }
  if (dom.panelJoystick) {
    dom.panelJoystick.classList.toggle('controls-locked', controlsLocked);
  }

  if (dom.matrixLockPill) {
    dom.matrixLockPill.className = `lock-pill ${controlsLocked ? 'locked' : 'unlocked'}`;
    dom.matrixLockPill.innerHTML = controlsLocked ? '🔒 AUTONOMOUS' : '🔓 MANUAL ACTIVE';
  }
  if (dom.joystickLockPill) {
    dom.joystickLockPill.className = `lock-pill ${controlsLocked ? 'locked' : 'unlocked'}`;
    dom.joystickLockPill.innerHTML = controlsLocked ? '🔒 LOCKED' : '🔓 ACTIVE';
  }

  // Enable / Disable matrix and preset buttons
  dom.matrixButtons.forEach(btn => {
    btn.disabled = controlsLocked;
  });
  dom.rotationButtons.forEach(btn => {
    btn.disabled = controlsLocked;
  });
  dom.presetButtons.forEach(btn => {
    btn.disabled = controlsLocked;
  });

  // If locking, reset joystick stick to center and clear keys
  if (controlsLocked) {
    state.activeKeys.clear();
    state.joystickActive = false;
    if (dom.joystickStick) {
      dom.joystickStick.style.transform = 'translate(0px, 0px)';
    }
    if (dom.teleAngle) dom.teleAngle.textContent = '0°';
    if (dom.teleIntensity) dom.teleIntensity.textContent = '0%';
  }
}

// ============================================================================
// BLUETOOTH LE HANDLER
// ============================================================================

dom.btnConnectBle.addEventListener('click', async () => {
  if (state.isConnected) {
    disconnectBle();
  } else {
    connectBle();
  }
});

async function connectBle() {
  if (!navigator.bluetooth) {
    alert('Web Bluetooth is not supported on this browser. Please use Google Chrome or Microsoft Edge.');
    return;
  }

  try {
    state.isConnecting = true;
    updateConnectionStatus('connecting', 'CONNECTING...');
    addLog('sys', 'BLE', 'Requesting Bluetooth device...');

    // Request Bluetooth Device with primary and fallback service discovery
    state.bluetoothDevice = await navigator.bluetooth.requestDevice({
      filters: [
        { namePrefix: BLE_CONFIG.DEVICE_PREFIX_1 },
        { namePrefix: BLE_CONFIG.DEVICE_PREFIX_2 },
        { services: [BLE_CONFIG.PRIMARY_SERVICE] }
      ],
      optionalServices: [
        BLE_CONFIG.PRIMARY_SERVICE,
        BLE_CONFIG.LEGACY_SERVICE
      ]
    }).catch(async () => {
      addLog('sys', 'BLE', 'Retrying discovery with all devices filter...');
      return await navigator.bluetooth.requestDevice({
        acceptAllDevices: true,
        optionalServices: [
          BLE_CONFIG.PRIMARY_SERVICE,
          BLE_CONFIG.LEGACY_SERVICE
        ]
      });
    });

    if (!state.bluetoothDevice) {
      throw new Error('Device selection was cancelled.');
    }

    const deviceName = state.bluetoothDevice.name || 'Dhruv_Arduino';
    addLog('sys', 'BLE', `Device selected: ${deviceName} (${state.bluetoothDevice.id})`);
    dom.statDevName.textContent = deviceName;

    // Listen for unexpected disconnection
    state.bluetoothDevice.addEventListener('gattserverdisconnected', onBleDisconnected);

    // Connect to GATT Server
    addLog('sys', 'BLE', 'Connecting to GATT server...');
    state.gattServer = await state.bluetoothDevice.gatt.connect();

    // Determine Service (Primary Dhruv Cable vs Legacy)
    let service = null;
    let isPrimaryService = false;

    try {
      addLog('sys', 'BLE', `Accessing Primary Service: ${BLE_CONFIG.PRIMARY_SERVICE}`);
      service = await state.gattServer.getPrimaryService(BLE_CONFIG.PRIMARY_SERVICE);
      isPrimaryService = true;
    } catch (ePrimary) {
      addLog('sys', 'BLE', `Primary service not found; trying fallback: ${BLE_CONFIG.LEGACY_SERVICE}`);
      service = await state.gattServer.getPrimaryService(BLE_CONFIG.LEGACY_SERVICE);
      isPrimaryService = false;
    }

    if (isPrimaryService) {
      // Unified characteristic for Write & Notify in H-drive-jetson-cable.ino
      addLog('sys', 'BLE', `Accessing Dhruv Characteristic: ${BLE_CONFIG.PRIMARY_CHAR}`);
      state.cmdCharacteristic = await service.getCharacteristic(BLE_CONFIG.PRIMARY_CHAR);
      state.logCharacteristic = state.cmdCharacteristic;
    } else {
      // Legacy separate characteristics
      addLog('sys', 'BLE', `Accessing Legacy TX: ${BLE_CONFIG.LEGACY_CMD_CHAR}`);
      state.cmdCharacteristic = await service.getCharacteristic(BLE_CONFIG.LEGACY_CMD_CHAR);
      try {
        state.logCharacteristic = await service.getCharacteristic(BLE_CONFIG.LEGACY_LOG_CHAR);
      } catch (_) {
        state.logCharacteristic = state.cmdCharacteristic;
      }
    }

    // Subscribe to Telemetry Notifications (Mirrors 100% of Serial Monitor Output)
    try {
      await state.logCharacteristic.startNotifications();
      state.logCharacteristic.addEventListener('characteristicvaluechanged', onBleTelemetryReceived);
      addLog('sys', 'BLE', 'Live serial monitor telemetry subscribed (RX notifications enabled).');
    } catch (rxErr) {
      addLog('err', 'BLE', `Telemetry subscription warning: ${rxErr.message}`);
    }

    state.isConnected = true;
    state.isConnecting = false;
    updateConnectionStatus('connected', 'CONNECTED');
    addLog('sys', 'BLE', `Connected to ${deviceName}! Querying Arduino mode...`);

    // Query Arduino mode status
    setTimeout(() => {
      sendCommand('MODE?', false);
    }, 350);

  } catch (err) {
    state.isConnecting = false;
    state.isConnected = false;
    updateConnectionStatus('disconnected', 'DISCONNECTED');
    addLog('err', 'BLE', `Connection failed: ${err.message}`);
  }
}

function disconnectBle() {
  if (!state.bluetoothDevice) return;

  try {
    addLog('sys', 'BLE', 'Disconnecting from UNO R4...');
    sendCommand('Q', false);

    if (state.bluetoothDevice.gatt.connected) {
      state.bluetoothDevice.gatt.disconnect();
    }
  } catch (err) {
    addLog('err', 'BLE', `Error during disconnect: ${err.message}`);
  }
  onBleDisconnected();
}

function onBleDisconnected() {
  state.isConnected = false;
  state.isConnecting = false;
  state.gattServer = null;
  state.cmdCharacteristic = null;
  state.logCharacteristic = null;

  updateConnectionStatus('disconnected', 'DISCONNECTED');
  dom.statDevName.textContent = '--';
  addLog('sys', 'BLE', 'Bluetooth connection closed. Motors stopped.');
}

function updateConnectionStatus(status, label) {
  dom.statusDot.className = `status-dot ${status}`;
  dom.statusLabel.textContent = label;

  if (status === 'connected') {
    dom.btnConnectBle.classList.add('connected');
    dom.btnConnectText.textContent = 'Disconnect';
  } else if (status === 'connecting') {
    dom.btnConnectBle.classList.remove('connected');
    dom.btnConnectText.textContent = 'Connecting...';
  } else {
    dom.btnConnectBle.classList.remove('connected');
    dom.btnConnectText.textContent = 'Connect UNO R4';
  }
}

/**
 * 100% Live Telemetry Receiver:
 * Logs all output printed to the Arduino serial monitor directly into the dashboard terminal.
 */
function onBleTelemetryReceived(event) {
  const value = event.target.value;
  const decoder = new TextDecoder('utf-8');
  const message = decoder.decode(value).trim();

  if (message.length > 0) {
    state.rxCount++;
    dom.statRxCount.textContent = state.rxCount;

    // Direct terminal output for every single line from Arduino
    addLog('rx', 'RX', message);

    // Synchronize mode state with Arduino feedback
    if (message.includes('Switched to MANUAL') || message.includes('Mode=MANUAL')) {
      updateModeUI(false);
    } else if (message.includes('Switched to AUTONOMOUS') || message.includes('Mode=AUTONOMOUS')) {
      updateModeUI(true);
    } else if (message.includes('[BLOCKED]')) {
      addLog('err', 'LOCK', '⚠️ Command dropped: Arduino is in AUTONOMOUS mode. Click the MODE button to switch to MANUAL.');
    }
  }
}

// ============================================================================
// COMMAND TRANSMISSION PIPELINE
// ============================================================================

async function sendCommand(rawCmd, logToConsole = true) {
  if (!rawCmd) return;
  const cleanCmd = rawCmd.trim();
  if (!cleanCmd) return;

  // If using Jetson format, convert basic directional tokens (N, S, etc.) to <DIR,SPEED,TIME>|<LED,MOOD>
  let cmdToSend = cleanCmd;
  if (state.useJetsonFormat && !cleanCmd.includes('|') && !cleanCmd.startsWith('<') && !cleanCmd.startsWith('MODE') && !cleanCmd.startsWith('T')) {
    const mapped = JETSON_COMMAND_MAP[cleanCmd.toUpperCase()];
    if (mapped) {
      cmdToSend = mapped;
    }
  }

  // Rate-limiting / deduplication check for continuous commands
  const now = Date.now();
  if (cmdToSend === state.lastSentCmd && (now - state.lastSendTime < 75)) {
    return;
  }
  state.lastSentCmd = cmdToSend;
  state.lastSendTime = now;

  state.currentCmd = cmdToSend;
  updateTelemetryDisplay(cleanCmd);

  if (logToConsole) {
    state.txCount++;
    dom.statTxCount.textContent = state.txCount;
    addLog('tx', 'TX', cmdToSend);
  }

  // Transmit over Bluetooth LE if connected
  if (state.isConnected && state.cmdCharacteristic) {
    try {
      const encoder = new TextEncoder();
      const payload = encoder.encode(cmdToSend);

      if (state.cmdCharacteristic.writeValueWithoutResponse) {
        await state.cmdCharacteristic.writeValueWithoutResponse(payload);
      } else {
        await state.cmdCharacteristic.writeValue(payload);
      }
    } catch (err) {
      addLog('err', 'ERR', `Failed to transmit command [${cmdToSend}]: ${err.message}`);
    }
  }
}

function updateTelemetryDisplay(cmd) {
  const upper = cmd.toUpperCase();
  const cmdMap = {
    'N': 'FORWARD (N) ➔ <FWD,180,0>',
    'S': 'BACKWARD (S) ➔ <REV,180,0>',
    'W': 'LEFT (W) ➔ <STRAFE_L,180,0>',
    'E': 'RIGHT (E) ➔ <STRAFE_R,180,0>',
    'NE': 'FWD-RIGHT (NE) ➔ <DIAG_FR,180,0>',
    'NW': 'FWD-LEFT (NW) ➔ <DIAG_FL,180,0>',
    'SE': 'BWD-RIGHT (SE) ➔ <DIAG_BR,180,0>',
    'SW': 'BWD-LEFT (SW) ➔ <DIAG_BL,180,0>',
    'CW': 'ROTATE CW (CW) ➔ <PIVOT_R,180,0>',
    'CCW': 'ROTATE CCW (CCW) ➔ <PIVOT_L,180,0>',
    'Q': 'STOP (Q) ➔ <STOP>',
    'X': 'STOP (X) ➔ <STOP>',
    'STOP': 'STOP ➔ <STOP>'
  };

  dom.teleCmd.textContent = cmdMap[upper] || upper;
}

// ============================================================================
// DIRECT COMMAND MATRIX & BUTTON CONTROLS
// ============================================================================

function initMatrixControls() {
  dom.chkHoldToMove.addEventListener('change', (e) => {
    state.holdToMove = e.target.checked;
    addLog('sys', 'CFG', `Hold-to-Move mode: ${state.holdToMove ? 'ENABLED' : 'DISABLED (Click to Trigger)'}`);
  });

  // Attach matrix buttons
  dom.matrixButtons.forEach((btn) => {
    const cmd = btn.getAttribute('data-cmd');

    btn.addEventListener('pointerdown', (e) => {
      // Safety block if in Autonomous mode
      if (state.isAutonomous) {
        addLog('err', 'LOCK', '⚠️ Manual controls are locked in AUTONOMOUS mode. Click MODE in the header to switch to MANUAL.');
        return;
      }

      e.preventDefault();
      btn.classList.add('is-active');
      sendCommand(cmd);
    });

    const releaseHandler = () => {
      if (!btn.classList.contains('is-active')) return;
      btn.classList.remove('is-active');

      if (state.holdToMove && cmd !== 'Q' && !state.isAutonomous) {
        sendCommand('Q');
      }
    };

    btn.addEventListener('pointerup', releaseHandler);
    btn.addEventListener('pointerleave', releaseHandler);
    btn.addEventListener('pointercancel', releaseHandler);
  });
}

// ============================================================================
// OMNIDIRECTIONAL JOYSTICK ENGINE
// ============================================================================

function initJoystickEngine() {
  const base = dom.joystickBase;
  const stick = dom.joystickStick;
  let isDragging = false;
  let stickPointerId = null;

  const baseRect = () => base.getBoundingClientRect();

  const handlePointerStart = (e) => {
    // Safety block if in Autonomous mode
    if (state.isAutonomous) {
      addLog('err', 'LOCK', '⚠️ Joystick is locked in AUTONOMOUS mode. Click MODE in the header to switch to MANUAL.');
      return;
    }

    e.preventDefault();
    isDragging = true;
    stickPointerId = e.pointerId;
    stick.setPointerCapture(e.pointerId);
    stick.classList.add('is-dragging');
    dom.joystickActiveBadge.classList.add('active');
    dom.joystickActiveBadge.textContent = 'ACTIVE';

    updateJoystickPosition(e.clientX, e.clientY);
  };

  const handlePointerMove = (e) => {
    if (!isDragging || state.isAutonomous) return;
    updateJoystickPosition(e.clientX, e.clientY);
  };

  const handlePointerEnd = () => {
    if (!isDragging) return;
    isDragging = false;
    try {
      if (stickPointerId !== null) stick.releasePointerCapture(stickPointerId);
    } catch (_) {}
    stickPointerId = null;

    stick.classList.remove('is-dragging');
    dom.joystickActiveBadge.classList.remove('active');
    dom.joystickActiveBadge.textContent = 'IDLE';

    stick.style.transform = `translate(0px, 0px)`;
    dom.teleAngle.textContent = `0°`;
    dom.teleIntensity.textContent = `0%`;

    if (!state.isAutonomous) {
      sendCommand('Q');
    }
    highlightMatrixButton(null);
  };

  base.addEventListener('pointerdown', handlePointerStart);
  window.addEventListener('pointermove', handlePointerMove);
  window.addEventListener('pointerup', handlePointerEnd);
  window.addEventListener('pointercancel', handlePointerEnd);

  function updateJoystickPosition(clientX, clientY) {
    const rect = baseRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const dx = clientX - centerX;
    const dy = clientY - centerY;

    const maxRadius = (rect.width / 2) - 36;
    const distance = Math.hypot(dx, dy);

    let clampedDist = distance;
    let stickX = dx;
    let stickY = dy;

    if (distance > maxRadius) {
      clampedDist = maxRadius;
      stickX = (dx / distance) * maxRadius;
      stickY = (dy / distance) * maxRadius;
    }

    stick.style.transform = `translate(${stickX}px, ${stickY}px)`;

    let angleRad = Math.atan2(-dy, dx);
    let angleDeg = Math.round((angleRad * 180) / Math.PI);
    if (angleDeg < 0) angleDeg += 360;

    const intensity = Math.min(100, Math.round((clampedDist / maxRadius) * 100));

    dom.teleAngle.textContent = `${angleDeg}°`;
    dom.teleIntensity.textContent = `${intensity}%`;

    // Deadzone threshold (18%)
    if (intensity < 18) {
      if (state.activeSector !== 'STOP') {
        state.activeSector = 'STOP';
        sendCommand('Q');
        highlightMatrixButton(null);
      }
      return;
    }

    const sector = mapAngleToSector(angleDeg);
    state.activeSector = sector;

    highlightMatrixButton(sector);
    sendCommand(sector);
  }

  function mapAngleToSector(deg) {
    if (deg >= 67.5 && deg < 112.5) return 'N';
    if (deg >= 22.5 && deg < 67.5) return 'NE';
    if (deg >= 337.5 || deg < 22.5) return 'E';
    if (deg >= 292.5 && deg < 337.5) return 'SE';
    if (deg >= 247.5 && deg < 292.5) return 'S';
    if (deg >= 202.5 && deg < 247.5) return 'SW';
    if (deg >= 157.5 && deg < 202.5) return 'W';
    if (deg >= 112.5 && deg < 157.5) return 'NW';
    return 'N';
  }
}

function highlightMatrixButton(cmd) {
  dom.matrixButtons.forEach(btn => {
    if (cmd && btn.getAttribute('data-cmd') === cmd) {
      btn.classList.add('is-active');
    } else {
      btn.classList.remove('is-active');
    }
  });
}

// ============================================================================
// KEYBOARD CONTROLS
// ============================================================================

function initKeyboardControls() {
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

    // Ignore manual driving keys in Autonomous mode
    if (state.isAutonomous) return;

    const key = e.key.toUpperCase();
    if (state.activeKeys.has(key)) return;
    state.activeKeys.add(key);

    evaluateKeyboardMovement();
  });

  window.addEventListener('keyup', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
    if (state.isAutonomous) return;

    const key = e.key.toUpperCase();
    state.activeKeys.delete(key);

    evaluateKeyboardMovement();
  });

  function evaluateKeyboardMovement() {
    const keys = state.activeKeys;

    // Emergency Stop Keys (Space or X)
    if (keys.has(' ') || keys.has('X')) {
      sendCommand('Q');
      highlightMatrixButton('Q');
      return;
    }

    // Rotations
    if (keys.has('Q') && !keys.has('E')) {
      sendCommand('CCW');
      highlightMatrixButton('CCW');
      return;
    }
    if (keys.has('E') && !keys.has('Q')) {
      sendCommand('CW');
      highlightMatrixButton('CW');
      return;
    }

    // Translational Directions
    const up = keys.has('W') || keys.has('ARROWUP');
    const down = keys.has('S') || keys.has('ARROWDOWN');
    const left = keys.has('A') || keys.has('ARROWLEFT');
    const right = keys.has('D') || keys.has('ARROWRIGHT');

    let cmd = null;

    if (up && right) cmd = 'NE';
    else if (up && left) cmd = 'NW';
    else if (down && right) cmd = 'SE';
    else if (down && left) cmd = 'SW';
    else if (up) cmd = 'N';
    else if (down) cmd = 'S';
    else if (left) cmd = 'W';
    else if (right) cmd = 'E';

    if (cmd) {
      sendCommand(cmd);
      highlightMatrixButton(cmd);
    } else {
      sendCommand('Q');
      highlightMatrixButton(null);
    }
  }
}

// ============================================================================
// LIVE SERIAL TELEMETRY LOGGER (ALWAYS ACTIVE & INTERACTIVE)
// ============================================================================

function initTerminalLogger() {
  dom.btnClearLog.addEventListener('click', () => {
    dom.terminalBody.innerHTML = '';
    addLog('sys', 'SYS', 'Console log cleared.');
  });

  dom.btnAutoScroll.addEventListener('click', () => {
    state.autoScroll = !state.autoScroll;
    dom.btnAutoScroll.classList.toggle('active', state.autoScroll);
    addLog('sys', 'SYS', `Auto-scroll: ${state.autoScroll ? 'ON' : 'OFF'}`);
  });

  dom.btnExportLog.addEventListener('click', () => {
    const entries = Array.from(dom.terminalBody.querySelectorAll('.log-entry')).map(el => {
      const time = el.querySelector('.log-time')?.textContent || '';
      const tag = el.querySelector('.log-tag')?.textContent || '';
      const text = el.querySelector('.log-text')?.textContent || '';
      return `${time} [${tag}] ${text}`;
    });

    if (entries.length === 0) {
      alert('Log is empty.');
      return;
    }

    const blob = new Blob([entries.join('\n')], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `uno_r4_telemetry_${new Date().toISOString().replace(/[:.]/g, '-')}.txt`;
    a.click();
    URL.revokeObjectURL(url);
  });

  dom.filterTabs.forEach(tab => {
    tab.addEventListener('click', () => {
      dom.filterTabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');

      const filter = tab.getAttribute('data-filter');
      state.currentFilter = filter;
      applyLogFilter(filter);
    });
  });

  // Manual Command Input (Always active so developer can query status or send custom commands)
  dom.terminalInputForm.addEventListener('submit', (e) => {
    e.preventDefault();
    const input = dom.manualCmdInput.value.trim();
    if (!input) return;

    sendCommand(input, true);
    dom.manualCmdInput.value = '';
  });
}

function addLog(type, tag, message) {
  if (!dom.terminalBody) return;

  const now = new Date();
  const timeStr = `${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}.${pad(now.getMilliseconds(), 3)}`;

  const entry = document.createElement('div');
  entry.className = `log-entry log-${type}`;
  entry.dataset.type = type;

  entry.innerHTML = `
    <span class="log-time">[${timeStr}]</span>
    <span class="log-tag tag-${type}">${tag}</span>
    <span class="log-text">${escapeHtml(message)}</span>
  `;

  if (state.currentFilter !== 'all' && state.currentFilter !== type) {
    entry.style.display = 'none';
  }

  dom.terminalBody.appendChild(entry);

  if (dom.terminalBody.children.length > 500) {
    dom.terminalBody.removeChild(dom.terminalBody.firstChild);
  }

  if (state.autoScroll) {
    dom.terminalContainer.scrollTop = dom.terminalContainer.scrollHeight;
    dom.terminalBody.scrollTop = dom.terminalBody.scrollHeight;
  }
}

function applyLogFilter(filter) {
  const entries = dom.terminalBody.querySelectorAll('.log-entry');
  entries.forEach(entry => {
    if (filter === 'all' || entry.dataset.type === filter) {
      entry.style.display = 'flex';
    } else {
      entry.style.display = 'none';
    }
  });
}

// Preset Buttons for Motor Tokens
function initPresetButtons() {
  dom.presetButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      if (state.isAutonomous) {
        addLog('err', 'LOCK', '⚠️ Preset buttons are locked in AUTONOMOUS mode.');
        return;
      }
      const token = btn.getAttribute('data-token');
      if (token) {
        sendCommand(token, true);
      }
    });
  });
}

// ============================================================================
// UTILITIES
// ============================================================================

function pad(num, size = 2) {
  let s = String(num);
  while (s.length < size) s = '0' + s;
  return s;
}

function escapeHtml(text) {
  const map = {
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#039;'
  };
  return String(text).replace(/[&<>"']/g, m => map[m]);
}
