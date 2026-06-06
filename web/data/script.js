/**
 =============================================================================
 This is file 8/12 – Web Interface (Part 3: JavaScript)
 =============================================================================
 
 Purpose: JavaScript for the SCARA robot web control interface.
 This file handles:
 - API calls to the robot's web server
 - Updating the status display
 - Handling user interactions
 - Drawing the workspace visualizer
 
 The robot's web server runs on the ESP32 at http://[robot-ip]/
 =============================================================================
 */

// ============================================================
// CONFIGURATION
// ============================================================

// Robot's IP address - automatically detected from current page
// The browser automatically knows the robot's IP from the page URL
const ROBOT_BASE_URL = window.location.origin;

// How often to update status (milliseconds)
// 500ms = 2 times per second
const STATUS_UPDATE_INTERVAL = 500;

// Robot physical parameters (for workspace visualizer)
// These match the values in robot_config.h
const MOTOR_SPACING = 50;     // Distance between motors in mm
const L1 = 70;               // Length of first arm segment in mm
const L2 = 100;              // Length of second arm segment in mm

// ============================================================
// STATE VARIABLES
// ============================================================

// Current robot state (updated by getStatus())
let robotState = {
    x: 0,              // X position in mm
    y: 0,              // Y position in mm
    angle_left: 0,      // Left motor angle in degrees
    angle_right: 0,    // Right motor angle in degrees
    pen_up: true,      // Pen state (true = up)
    homed: false       // Whether homing has been done
};

// Flag to track if status update loop is running
let statusUpdateInterval = null;

// ============================================================
// INITIALIZATION
// ============================================================

/**
 * Initialize the web interface when the page loads.
 * This function is called automatically when the HTML is parsed.
 */
function init() {
    console.log('SCARA Robot Web Interface initializing...');
    console.log('Robot URL:', ROBOT_BASE_URL);
    
    // Start periodic status updates
    startStatusUpdates();
    
    // Initialize the workspace visualizer
    initWorkspace();
    
    // Set up file input handler for G-code
    setupFileInput();
    
    // Update connection status
    updateConnectionStatus();
    
    console.log('Initialization complete');
}

/**
 * Start the periodic status update loop.
 * Calls getStatus() every STATUS_UPDATE_INTERVAL milliseconds.
 */
function startStatusUpdates() {
    // Clear any existing interval (safety check)
    if (statusUpdateInterval) {
        clearInterval(statusUpdateInterval);
    }
    
    // Get initial status immediately
    getStatus();
    
    // Then update every STATUS_UPDATE_INTERVAL ms
    statusUpdateInterval = setInterval(getStatus, STATUS_UPDATE_INTERVAL);
}

/**
 * Update the connection status indicator in the header.
 */
function updateConnectionStatus() {
    // Try to fetch the status endpoint
    fetch(`${ROBOT_BASE_URL}/status`)
        .then(response => {
            // Check if we got a valid response
            if (response.ok) {
                // Connection is good - show success
                const statusEl = document.getElementById('connection-status');
                statusEl.textContent = '🟢 Connected';
                statusEl.style.color = '#4CAF50';  // Green
            } else {
                // Server responded but with error
                const statusEl = document.getElementById('connection-status');
                statusEl.textContent = '🟡 Server Error';
                statusEl.style.color = '#ff9800';  // Orange
            }
        })
        .catch(error => {
            // Network error - can't reach server
            const statusEl = document.getElementById('connection-status');
            statusEl.textContent = '🔴 Disconnected';
            statusEl.style.color = '#f44336';  // Red
            console.log('Connection check failed:', error.message);
        });
}

// ============================================================
// STATUS FUNCTIONS
// ============================================================

/**
 * Fetch current robot status from the server.
 * Updates the display with position, angles, and pen state.
 */
async function getStatus() {
    try {
        // Make GET request to /status endpoint
        const response = await fetch(`${ROBOT_BASE_URL}/status`);
        
        // Check if response is OK
        if (!response.ok) {
            throw new Error(`HTTP error: ${response.status}`);
        }
        
        // Parse JSON response
        const data = await response.json();
        
        // Update our local state
        if (data.success) {
            robotState.x = data.x || 0;
            robotState.y = data.y || 0;
            robotState.angle_left = data.angle_left || 0;
            robotState.angle_right = data.angle_right || 0;
            robotState.pen_up = data.pen_up !== undefined ? data.pen_up : true;
            robotState.homed = data.homed || false;
            
            // Update the display
            updateStatusDisplay();
            
            // Update the workspace visualizer
            updateWorkspace();
        }
    } catch (error) {
        // Log errors but don't show to user (they're already seeing connection status)
        console.log('Status fetch failed:', error.message);
    }
}

/**
 * Update all status display elements with current values.
 * This is called every time we get new status from the robot.
 */
function updateStatusDisplay() {
    // Update X position
    const posX = document.getElementById('pos-x');
    if (posX) {
        posX.textContent = `${robotState.x.toFixed(1)} mm`;
    }
    
    // Update Y position
    const posY = document.getElementById('pos-y');
    if (posY) {
        posY.textContent = `${robotState.y.toFixed(1)} mm`;
    }
    
    // Update left angle
    const angleLeft = document.getElementById('angle-left');
    if (angleLeft) {
        angleLeft.textContent = `${robotState.angle_left.toFixed(1)}°`;
    }
    
    // Update right angle
    const angleRight = document.getElementById('angle-right');
    if (angleRight) {
        angleRight.textContent = `${robotState.angle_right.toFixed(1)}°`;
    }
    
    // Update pen state
    const penState = document.getElementById('pen-state');
    if (penState) {
        if (robotState.pen_up) {
            penState.textContent = 'UP ⬆️';
            penState.style.backgroundColor = '#4CAF50';  // Green
        } else {
            penState.textContent = 'DOWN ⬇️';
            penState.style.backgroundColor = '#f44336';   // Red
        }
    }
}

// ============================================================
// MOVEMENT FUNCTIONS
// ============================================================

/**
 * Move the robot to an absolute (x, y) position.
 * Sends a POST request to the /move endpoint.
 * 
 * @param {number} x - Target X position in millimeters
 * @param {number} y - Target Y position in millimeters
 */
async function moveTo(x, y) {
    showStatus('info', `Moving to (${x.toFixed(1)}, ${y.toFixed(1)})...`);
    
    try {
        // Send POST request with JSON body
        const response = await fetch(`${ROBOT_BASE_URL}/move`, {
            method: 'POST',                                    // HTTP method
            headers: { 'Content-Type': 'application/json' }, // Tell server we're sending JSON
            body: JSON.stringify({ x: x, y: y })             // Convert object to JSON string
        });
        
        // Parse response
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', `Move command sent: (${x.toFixed(1)}, ${y.toFixed(1)})`);
        } else {
            showStatus('error', `Move failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `Move error: ${error.message}`);
    }
}

/**
 * Move the robot relatively from current position.
 * Sends a POST request to the /moverel endpoint.
 * 
 * @param {number} dx - Change in X position in millimeters (positive = right)
 * @param {number} dy - Change in Y position in millimeters (positive = up)
 */
async function relativeMove(dx, dy) {
    showStatus('info', `Moving relative (${dx.toFixed(1)}, ${dy.toFixed(1)})...`);
    
    try {
        const response = await fetch(`${ROBOT_BASE_URL}/moverel`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ dx: dx, dy: dy })
        });
        
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', `Relative move: (${dx.toFixed(1)}, ${dy.toFixed(1)})`);
        } else {
            showStatus('error', `Move failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `Move error: ${error.message}`);
    }
}

/**
 * Control the pen (raise or lower).
 * Sends a POST request to the /pen endpoint.
 * 
 * @param {boolean} up - true to raise pen, false to lower
 */
async function setPen(up) {
    const state = up ? 'up' : 'down';
    showStatus('info', `Setting pen ${state}...`);
    
    try {
        const response = await fetch(`${ROBOT_BASE_URL}/pen`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ state: state })
        });
        
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', `Pen is now ${state}`);
            // Update local state immediately (optimistic update)
            robotState.pen_up = up;
            updateStatusDisplay();
        } else {
            showStatus('error', `Pen control failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `Pen error: ${error.message}`);
    }
}

/**
 * Run the homing sequence.
 * Sends a POST request to the /home endpoint.
 */
async function home() {
    showStatus('info', 'Running homing sequence...');
    
    try {
        const response = await fetch(`${ROBOT_BASE_URL}/home`, {
            method: 'POST'
        });
        
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', 'Homing complete!');
            // Reset position display to 0
            robotState.x = 0;
            robotState.y = 0;
            updateStatusDisplay();
        } else {
            showStatus('error', `Homing failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `Homing error: ${error.message}`);
    }
}

/**
 * Jog (manually move) the robot by a small amount.
 * Sends a POST request to the /jog endpoint.
 * 
 * @param {string} axis - 'x' or 'y'
 * @param {number} steps - Number of steps (positive or negative)
 */
async function jog(axis, steps) {
    // Validate input
    if (axis !== 'x' && axis !== 'y') {
        showStatus('error', 'Invalid axis. Use "x" or "y".');
        return;
    }
    
    try {
        const response = await fetch(`${ROBOT_BASE_URL}/jog`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ axis: axis, steps: steps })
        });
        
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', `Jogged ${axis} by ${steps}`);
        } else {
            showStatus('error', `Jog failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `Jog error: ${error.message}`);
    }
}

/**
 * Get the selected step size from the radio buttons.
 * Returns the value in millimeters.
 * 
 * @returns {number} Selected step size
 */
function getStepSize() {
    // Get all radio buttons with name "stepSize"
    const radios = document.getElementsByName('stepSize');
    
    // Find the selected one
    for (const radio of radios) {
        if (radio.checked) {
            // Convert string to number
            return parseFloat(radio.value);
        }
    }
    
    // Default to 1mm if nothing selected
    return 1;
}

// ============================================================
// CALIBRATION FUNCTIONS
// ============================================================

/**
 * Set the current encoder angle as zero for a motor.
 * Sends a POST request to the /calibrate/encoder endpoint.
 * 
 * @param {number} motorId - Motor ID (0 = left, 1 = right)
 */
async function calibrateEncoder(motorId) {
    const motorName = motorId === 0 ? 'Left' : 'Right';
    showStatus('info', `Calibrating ${motorName} encoder...`);
    
    try {
        const response = await fetch(`${ROBOT_BASE_URL}/calibrate/encoder`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ motor: motorId })
        });
        
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', `${motorName} encoder zero set!`);
        } else {
            showStatus('error', `Calibration failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `Calibration error: ${error.message}`);
    }
}

/**
 * Update PID gains from the sliders.
 * Sends a POST request to the /calibrate/pid endpoint.
 */
async function updatePID() {
    // Read values from sliders
    const kp = parseFloat(document.getElementById('kp-slider').value);
    const ki = parseFloat(document.getElementById('ki-slider').value);
    const kd = parseFloat(document.getElementById('kd-slider').value);
    
    showStatus('info', 'Updating PID gains...');
    
    try {
        const response = await fetch(`${ROBOT_BASE_URL}/calibrate/pid`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ kp: kp, ki: ki, kd: kd })
        });
        
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', `PID updated: Kp=${kp.toFixed(2)}, Ki=${ki.toFixed(2)}, Kd=${kd.toFixed(2)}`);
        } else {
            showStatus('error', `PID update failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `PID error: ${error.message}`);
    }
}

/**
 * Update the displayed value when a slider changes.
 * 
 * @param {string} param - Which parameter ('kp', 'ki', or 'kd')
 */
function updateSliderValue(param) {
    // Get the slider element
    const slider = document.getElementById(`${param}-slider`);
    // Get the value display element
    const valueDisplay = document.getElementById(`${param}-value`);
    
    // Update the displayed value with 2 decimal places
    if (slider && valueDisplay) {
        valueDisplay.textContent = parseFloat(slider.value).toFixed(2);
    }
}

/**
 * Step a motor directly by a number of steps.
 * For testing individual motors.
 * 
 * @param {number} motorId - Motor ID (0 = left, 1 = right)
 * @param {number} steps - Number of steps (positive or negative)
 */
async function motorRaw(motorId, steps) {
    const motorName = motorId === 0 ? 'Left' : 'Right';
    
    try {
        const response = await fetch(`${ROBOT_BASE_URL}/motor_raw`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ motor: motorId, steps: steps })
        });
        
        const data = await response.json();
        
        if (data.success) {
            showStatus('success', `${motorName} motor: ${steps > 0 ? '+' : ''}${steps} steps`);
        } else {
            showStatus('error', `Motor control failed: ${data.message}`);
        }
    } catch (error) {
        showStatus('error', `Motor error: ${error.message}`);
    }
}

// ============================================================
// G-CODE FUNCTIONS
// ============================================================

/**
 * Set up the file input handler for G-code files.
 * Called during initialization.
 */
function setupFileInput() {
    // Get the file input element
    const fileInput = document.getElementById('gcode-file');
    
    // Add event listener for when a file is selected
    if (fileInput) {
        fileInput.addEventListener('change', function(event) {
            // Get the selected file
            const file = event.target.files[0];
            
            if (file) {
                // Read the file contents
                const reader = new FileReader();
                
                // When file is loaded, display it
                reader.onload = function(e) {
                    const content = e.target.result;
                    const preview = document.getElementById('gcode-content');
                    if (preview) {
                        // Show first 1000 characters
                        preview.textContent = content.substring(0, 1000) + 
                            (content.length > 1000 ? '\n... (truncated)' : '');
                    }
                };
                
                // Read the file as text
                reader.readAsText(file);
            }
        });
    }
}

/**
 * Send G-code file to the robot.
 * Reads the uploaded file and sends it line by line.
 */
async function sendGcode() {
    // Get the file input element
    const fileInput = document.getElementById('gcode-file');
    
    // Check if a file was selected
    if (!fileInput || !fileInput.files[0]) {
        showStatus('error', 'Please select a G-code file first');
        return;
    }
    
    // Read the file
    const file = fileInput.files[0];
    const reader = new FileReader();
    
    showStatus('info', `Reading ${file.name}...`);
    
    reader.onload = async function(e) {
        // Get file contents
        const content = e.target.result;
        
        // Split into lines
        const lines = content.split('\n');
        
        // Filter out empty lines and comments
        const gcodeLines = lines
            .map(line => line.trim())                           // Remove whitespace
            .filter(line => line.length > 0)                   // Remove empty lines
            .filter(line => !line.startsWith(';'))              // Remove comments
            .filter(line => !line.startsWith('('));            // Remove parenthesis comments
        
        showStatus('info', `Sending ${gcodeLines.length} G-code lines...`);
        
        // Track progress
        let sent = 0;
        const total = gcodeLines.length;
        
        // Send each line
        for (const line of gcodeLines) {
            // Skip empty lines
            if (!line.trim()) continue;
            
            try {
                // Parse the G-code line
                // This is a simplified parser - real G-code parsing would be more complex
                const result = await sendGcodeLine(line);
                
                if (result.success) {
                    sent++;
                    // Update status every 10 lines
                    if (sent % 10 === 0 || sent === total) {
                        showStatus('info', `G-code progress: ${sent}/${total} lines`);
                    }
                } else {
                    showStatus('error', `G-code error at line ${sent + 1}: ${result.message}`);
                    break;  // Stop on error
                }
            } catch (error) {
                showStatus('error', `G-code error: ${error.message}`);
                break;
            }
        }
        
        showStatus('success', `G-code complete! Sent ${sent} lines.`);
    };
    
    reader.onerror = function() {
        showStatus('error', 'Error reading file');
    };
    
    // Read the file
    reader.readAsText(file);
}

/**
 * Send a single G-code line to the robot.
 * This is a placeholder - real implementation would parse G-code commands.
 * 
 * @param {string} line - Single line of G-code
 * @returns {Object} Result with success flag and message
 */
async function sendGcodeLine(line) {
    // This is a simplified implementation
    // Real G-code parsing would convert commands like:
    // G0 X10 Y20 -> moveTo(10, 20)
    // G1 X30 Y40 F100 -> moveTo(30, 40) with speed 100
    // M3 -> pen down
    // M5 -> pen up
    
    // For now, just log and return success
    console.log('G-code:', line);
    
    return { success: true, message: 'OK' };
}

// ============================================================
// WORKSPACE VISUALIZER
// ============================================================

/**
 * Initialize the workspace visualizer.
 * Draws the reachable area based on arm lengths.
 */
function initWorkspace() {
    // Get SVG elements
    const svg = document.getElementById('workspace-svg');
    if (!svg) return;
    
    // Calculate scaling
    // We want to fit the full reach of the robot in the 400x400 viewBox
    const maxReach = L1 + L2;  // Maximum reach = 70 + 100 = 170mm
    const scale = 200 / maxReach;  // Pixels per mm
    
    // Calculate motor positions in pixels
    const leftMotorX = 100;  // Fixed position in SVG
    const rightMotorX = 300; // Fixed position in SVG
    const centerY = 200;     // Center Y position
    
    // Calculate radius for reach circles
    const reachRadius = maxReach * scale;
    
    // Update reach circles
    const reachLeft = document.getElementById('reach-left');
    const reachRight = document.getElementById('reach-right');
    
    if (reachLeft) {
        reachLeft.setAttribute('cx', leftMotorX);
        reachLeft.setAttribute('cy', centerY);
        reachLeft.setAttribute('r', reachRadius);
    }
    
    if (reachRight) {
        reachRight.setAttribute('cx', rightMotorX);
        reachRight.setAttribute('cy', centerY);
        reachRight.setAttribute('r', reachRadius);
    }
}

/**
 * Update the workspace visualizer with current position.
 * Draws the arms and position marker.
 */
function updateWorkspace() {
    // Calculate scaling (same as initWorkspace)
    const maxReach = L1 + L2;
    const scale = 200 / maxReach;
    
    // SVG coordinates (400x400 viewBox)
    const svgWidth = 400;
    const svgHeight = 400;
    const centerX = svgWidth / 2;
    const centerY = svgHeight / 2;
    
    // Motor positions in SVG coordinates
    const motorSpacingPx = MOTOR_SPACING * scale;
    const leftMotorX = centerX - motorSpacingPx / 2;
    const rightMotorX = centerX + motorSpacingPx / 2;
    const motorY = centerY;
    
    // Robot position in SVG coordinates
    // Shift so (0,0) is at center of SVG
    const robotX = centerX + robotState.x * scale;
    const robotY = centerY - robotState.y * scale;  // Flip Y axis
    
    // Calculate elbow position using inverse kinematics (simplified)
    // This is a basic visualization - real IK would be more accurate
    const elbowY = motorY - L1 * scale * 0.7;  // Approximate elbow height
    
    // Update arm links
    const arm1 = document.getElementById('arm1');
    const arm2 = document.getElementById('arm2');
    
    if (arm1) {
        // Left arm (motor to elbow)
        arm1.setAttribute('x1', leftMotorX);
        arm1.setAttribute('y1', motorY);
        arm1.setAttribute('x2', centerX);
        arm1.setAttribute('y2', elbowY);
    }
    
    if (arm2) {
        // Right arm (elbow to end effector)
        arm2.setAttribute('x1', centerX);
        arm2.setAttribute('y1', elbowY);
        arm2.setAttribute('x2', robotX);
        arm2.setAttribute('y2', robotY);
    }
    
    // Update position marker
    const marker = document.getElementById('position-marker');
    if (marker) {
        marker.setAttribute('transform', `translate(${robotX}, ${robotY})`);
    }
}

// ============================================================
// STATUS MESSAGE DISPLAY
// ============================================================

/**
 * Show a status message in the status bar.
 * 
 * @param {string} type - Message type: 'success', 'error', 'info'
 * @param {string} message - The message to display
 */
function showStatus(type, message) {
    // Get the status message element
    const statusEl = document.getElementById('status-message');
    
    if (statusEl) {
        // Set the message text
        statusEl.textContent = message;
        
        // Remove existing type classes
        statusEl.classList.remove('success', 'error', 'info');
        
        // Add the new type class
        if (type) {
            statusEl.classList.add(type);
        }
        
        // Log to console as well
        console.log(`[${type.toUpperCase()}]`, message);
        
        // Auto-hide success messages after 3 seconds
        if (type === 'success') {
            setTimeout(() => {
                if (statusEl.textContent === message) {
                    statusEl.textContent = 'Ready';
                    statusEl.classList.remove('success');
                }
            }, 3000);
        }
    }
}

// ============================================================
// INITIALIZATION ON PAGE LOAD
// ============================================================

// When the DOM is fully loaded, initialize the interface
// DOMContentLoaded fires when HTML is parsed (before images/styles finish)
document.addEventListener('DOMContentLoaded', init);

// If DOM already loaded, init immediately
if (document.readyState === 'complete' || document.readyState === 'interactive') {
    init();
}