import os
import signal
import subprocess
from flask import Flask, render_template_string, jsonify

app = Flask(__name__)
ros_process = None

INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
    <title>Hexapod Field HUD</title>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.0/nipplejs.min.js"></script>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 10px; text-align: center; overflow: hidden; }
        h2 { margin: 5px 0 15px 0; color: #38bdf8; font-size: 24px; text-transform: uppercase; letter-spacing: 2px; }
        
        #video-container { margin: 10px auto; width: 100%; max-width: 600px; height: 240px; background: #1e293b; border: 2px solid #334155; border-radius: 8px; overflow: hidden; display: flex; align-items: center; justify-content: center; }
        #video-container img { width: 100%; height: 100%; object-fit: cover; }
        
        #joystick-zone { position: relative; width: 100%; max-width: 600px; height: 220px; background: #1e293b; border-radius: 8px; margin: 15px auto; border: 2px solid #334155; transition: opacity 0.3s; }
        .joystick-disabled { opacity: 0.3; pointer-events: none; }
        
        .controls, .mode-controls { display: flex; justify-content: center; gap: 15px; margin-bottom: 15px; }
        
        button { padding: 12px 20px; font-size: 16px; font-weight: bold; border: none; border-radius: 6px; cursor: pointer; text-transform: uppercase; flex: 1; max-width: 200px; transition: background 0.2s; }
        
        .btn-start { background: #10b981; color: white; box-shadow: 0 4px 6px -1px rgba(16, 185, 129, 0.4); }
        .btn-start:active { background: #059669; }
        .btn-stop { background: #ef4444; color: white; box-shadow: 0 4px 6px -1px rgba(239, 68, 68, 0.4); }
        .btn-stop:active { background: #dc2626; }
        
        .btn-mode { background: #334155; color: #cbd5e1; border: 2px solid #475569; }
        .btn-mode.active { background: #3b82f6; color: white; border-color: #60a5fa; box-shadow: 0 0 10px rgba(59, 130, 246, 0.5); }
        
        #status { font-weight: bold; padding: 8px; border-radius: 4px; background: #334155; display: inline-block; min-width: 250px; margin-bottom: 10px; }
        .status-idle { color: #94a3b8; }
        .status-booting { color: #f59e0b; }
        .status-active { color: #10b981; }
    </style>
</head>
<body>
    <h2>Field Control HUD</h2>
    
    <div class="controls">
        <button class="btn-start" onclick="sendCommand('/start')">Boot Stack</button>
        <button class="btn-stop" onclick="sendCommand('/stop')">E-Stop</button>
    </div>
    
    <div class="mode-controls">
        <button id="btn-teleop" class="btn-mode active" onclick="setMode('teleop')">Teleop</button>
        <button id="btn-follow" class="btn-mode" onclick="setMode('follow')">Auto Follow</button>
    </div>
    
    <div id="status" class="status-idle">System Offline</div>

    <div id="video-container">
        <img id="video-stream" src="" alt="Awaiting ROS 2 Video Feed..." />
    </div>

    <div id="joystick-zone"></div>

    <script>
        let ros = null;
        let cmdVel = null;
        let modeTopic = null;
        let currentMode = 'teleop';
        let pi_ip = window.location.hostname;

        function setStatus(text, className) {
            let el = document.getElementById('status');
            el.innerText = text;
            el.className = className;
        }

        function setMode(mode) {
            currentMode = mode;
            
            // Update button styles
            document.getElementById('btn-teleop').className = mode === 'teleop' ? 'btn-mode active' : 'btn-mode';
            document.getElementById('btn-follow').className = mode === 'follow' ? 'btn-mode active' : 'btn-mode';
            
            // Dim/Disable joystick if in follow mode
            document.getElementById('joystick-zone').className = mode === 'teleop' ? '' : 'joystick-disabled';
            
            // Publish mode to ROS 2
            if (modeTopic) {
                let msg = new ROSLIB.Message({ data: mode });
                modeTopic.publish(msg);
            }
        }

        function sendCommand(endpoint) {
            if (endpoint === '/start') setStatus("Booting ROS 2...", "status-booting");
            
            fetch(endpoint)
                .then(response => response.json())
                .then(data => {
                    if(endpoint === '/start') {
                        setTimeout(connectROS, 10000); 
                    } else {
                        setStatus("System Offline", "status-idle");
                        document.getElementById('video-stream').src = "";
                        if (ros) ros.close();
                    }
                })
                .catch(err => console.log("Fetch error: ", err));
        }

        function connectROS() {
            ros = new ROSLIB.Ros({ url: 'ws://' + pi_ip + ':9090' });
            
            ros.on('connection', function() {
                setStatus("ROS 2 Active & Linked", "status-active");
                
                // Mjpeg type set here
                document.getElementById('video-stream').src = "http://" + pi_ip + ":8080/stream?topic=/camera/depth/image_raw&type=mjpeg";
                
                cmdVel = new ROSLIB.Topic({
                    ros : ros,
                    name : '/cmd_vel',
                    messageType : 'geometry_msgs/Twist'
                });
                
                modeTopic = new ROSLIB.Topic({
                    ros : ros,
                    name : '/robot_mode',
                    messageType : 'std_msgs/String'
                });
                
                // Ensure initial mode is broadcasted once connected
                setMode(currentMode);
            });

            ros.on('error', function(error) {
                setStatus("WebSocket Error - Retrying...", "status-booting");
                setTimeout(connectROS, 3000);
            });
        }

        var joystick = nipplejs.create({
            zone: document.getElementById('joystick-zone'),
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#38bdf8'
        });

        joystick.on('move', function (evt, data) {
            // Ignore joystick input if autonomous follow mode is active
            if (!cmdVel || currentMode !== 'teleop') return;
            
            let max_linear = 0.3; 
            let max_angular = 0.5; 
            
            let linear = (data.distance / 50) * max_linear * Math.sin(data.angle.radian);
            let angular = -(data.distance / 50) * max_angular * Math.cos(data.angle.radian);

            let twist = new ROSLIB.Message({
                linear : { x : linear, y : 0.0, z : 0.0 },
                angular : { x : 0.0, y : 0.0, z : angular }
            });
            cmdVel.publish(twist);
        });

        joystick.on('end', function () {
            if (!cmdVel || currentMode !== 'teleop') return;
            let twist = new ROSLIB.Message({
                linear : { x : 0.0, y : 0.0, z : 0.0 },
                angular : { x : 0.0, y : 0.0, z : 0.0 }
            });
            cmdVel.publish(twist);
        });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(INDEX_HTML)

@app.route('/start')
def start_ros():
    global ros_process
    if ros_process is None or ros_process.poll() is not None:
        cmd = """
        source /opt/ros/lyrical/setup.bash &&
        source /home/raspberry/hexapod_ws/install/setup.bash &&
        ros2 launch hexapod_perception hexapod_robot.launch.py &
        ros2 run rosbridge_server rosbridge_websocket &
        ros2 run web_video_server web_video_server
        """
        ros_process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid, executable='/bin/bash')
        return jsonify({"status": "started", "message": "ROS 2 Stack Booting..."})
    return jsonify({"status": "running", "message": "Already running."})

@app.route('/stop')
def stop_ros():
    global ros_process
    if ros_process is not None:
        try:
            os.killpg(os.getpgid(ros_process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        ros_process = None
        return jsonify({"status": "stopped", "message": "ROS 2 Stack Killed."})
    return jsonify({"status": "stopped", "message": "Nothing is running."})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

