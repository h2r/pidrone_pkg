/**
* Setup all visualization elements when the page is loaded.
*/

function empty(element) {
    while (element.firstChild) {
      element.removeChild(element.firstChild);
    }
}

function printProperties(obj) {
    for(var propt in obj){
      console.log(propt + ': ' + obj[propt]);
    }
}

function myround(number, precision) {
  var factor = Math.pow(10, precision);
  var tempNumber = number * factor;
  var roundedTempNumber = Math.round(tempNumber);
  return roundedTempNumber / factor;
};

var markerClient;
var ros;
var modepub;
var modeMsg;
var positionMsg;
var twistMsg;
var poseMsg;
var positionPub;
var positionControlPub;
var velocityControlPub;
var heartbeatPub;
var heightChart;
var windowSize = 5;
var gotFirstHeight = false;
var startTime;
var heightChartPaused = false;
var showingUkfAnalysis = false;
var spanningFullWindow = false;

function closeSession(){
  console.log("Closing connections.");
  ros.disconnect();
  return false;
}

/* This code runs when you load the page */
function init() {
    // Connect to ROS.
    var url = 'ws://' + document.getElementById('hostname').value + ':9090'
    ros = new ROSLIB.Ros({
        url : url
    });

    var velocityBtn = document.getElementById('velocityBtn');
    velocityBtn.addEventListener("click", publishVelocityMode, false);

    var positionBtn = document.getElementById('positionBtn');
    positionBtn.addEventListener("click", publishPositionMode, false);

    ros.on('error', function(error) {
      console.log('ROS Master:  Error, check console.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML='Error detected; check console.';
      $('#statusMessage').addClass('alert-danger').removeClass('alert-success');
    });

    ros.on('connection', function() {
      console.log('ROS Master:  Connected.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML="Connected";
      $('#statusMessage').addClass('alert-success').removeClass('alert-danger');
    });

    ros.on('close', function() {
      console.log('ROS Master:  Connection closed.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML="Disconnected";
      $('#statusMessage').addClass('alert-danger').removeClass('alert-success');
    });

    /*
     * ROS Messages
     */

    modeMsg = new ROSLIB.Message({
      mode: "DISARMED",
     });

    emptyMsg = new ROSLIB.Message({
    });

    positionMsg = new ROSLIB.Message({
        // default is velocity mode
        data : false
    });

    poseMsg = new ROSLIB.Message({
        position : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        },
        orientation : {
            x : 0.0,
            y : 0.0,
            z : 0.0,
            w : 0.0
        }
    });

    twistMsg = new ROSLIB.Message({
        linear : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        },
        angular : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        }
    });

    /*
     * ROS Publishers
     */

    modepub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/desired/mode',
      messageType : 'pidrone_pkg/Mode'
    });

    heartbeatPub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/heartbeat/web_interface',
      messageType : 'std_msgs/Empty'
    });

    setInterval(function(){
      heartbeatPub.publish(emptyMsg);
      //console.log("heartbeat");
    }, 1000);

    positionPub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/position_control',
        messageType : 'std_msgs/Bool'
    });

    velocityControlPub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/desired/twist',
        messageType : 'geometry_msgs/Twist'
    });

    positionControlPub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/desired/pose',
        messageType : 'geometry_msgs/Pose'
    });

    resetpub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/reset_transform',
      messageType : 'std_msgs/Empty'
    });

    mappub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/map',
        messageType : 'std_msgs/Empty'
    })

    togglepub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/toggle_transform',
      messageType : 'std_msgs/Empty'
    });

    /*
     * ROS Subscribers
     */

    positionSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/position_control',
        messageType : 'std_msgs/Bool',
        queue_length : 2,
        throttle_rate : 80
    });

    // TODO: Merge with code that has Battery.msg
    // (published from flight controller node)
    batterysub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/battery',
      messageType : 'pidrone_pkg/Battery',
      queue_length : 2,
      throttle_rate : 2
    });

    irsub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/infrared',
      messageType : 'sensor_msgs/Range',
      queue_length : 2,
      throttle_rate : 80
    });

    ukf2dsub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state/ukf_2d',
        messageType : 'pidrone_pkg/State',
        queue_length : 2,
        throttle_rate : 80
    });
    
    ukf7dsub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state/ukf_7d',
        messageType : 'pidrone_pkg/State',
        queue_length : 2,
        throttle_rate : 80
    });

    cameraPoseSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/picamera/pose',
        messageType : 'geometry_msgs/PoseStamped',
        queue_length : 2,
        throttle_rate : 80
    });

    emaIrSub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/state/ema',
      messageType : 'pidrone_pkg/State',
      queue_length : 2,
      throttle_rate : 80
    });

    stateGroundTruthSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state/ground_truth',
        messageType : 'pidrone_pkg/StateGroundTruth',
        queue_length : 2,
        throttle_rate : 80
    });
    
    ukfStatsSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/ukf_stats',
        messageType : 'pidrone_pkg/UkfStats',
        queue_length : 2,
        throttle_rate : 80
    });

    /*
     * ROS Subscriber Callbacks
     */

     positionSub.subscribe(function(message) {
        var position = message.data;
        var text = "";
        if (position) {
            text = "Position Mode";
        } else {
            text = "Velocity Mode";
        }
        element = document.getElementById("position_state");
        element.textContent = text;
     });

    batterysub.subscribe(function(message) {
      //printProperties(message);
      var mynumber = myround(message.vbat, 2);
      document.getElementById('vbat').innerHTML=mynumber
      if (message.vbat <= 11.3) {
        document.getElementById('vbat').innerHTML=mynumber + " EMPTY!";
        $('#vbat').addClass('alert-danger').removeClass('alert-success');
      } else {
        document.getElementById('vbat').innerHTML=mynumber;
        $('#vbat').addClass('alert-success').removeClass('alert-danger');
      }

    });

    var heightChartMinTime;
    var heightChartMaxTime;
    irsub.subscribe(function(message) {
      //printProperties(message);
      //console.log("Range: " + message.range);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          heightChartMinTime = tVal - windowSize;
          heightChartMaxTime = tVal;
          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (rawIrData.length > 0 &&
                 (tVal - rawIrData[0].x > windowSize)) {
              rawIrData.splice(0, 1);
          }
      }
      // Add new range reading to end of the data array
      // x-y pair
      var xyPair = {
          x: tVal,
          y: message.range
      }
      rawIrData.push(xyPair)
      if (!heightChartPaused && !showingUkfAnalysis) {
          heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[0].data = rawIrData.slice();
          heightChart.update();
      } else if (!showingUkfAnalysis) {
          pulseIr();
      }
      //console.log("Data: " + heightChart.data.datasets[0].data);
      //console.log('tVal: ' + tVal)
    });

    function ukfCallback(message) {
      //printProperties(message);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          // Avoid changing axis limits too often, to avoid shaky plotting?
          // heightChartMinTime = tVal - windowSize;
          // heightChartMaxTime = tVal;

          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (ukfData.length > 0 &&
                 (tVal - ukfData[0].x > windowSize)) {
              ukfData.splice(0, 1);
              ukfPlusSigmaData.splice(0, 1);
              ukfMinusSigmaData.splice(0, 1);
          }
      }
      // Add new height estimate to end of the data array
      // x-y pair
      var zEstimate = message.pose_with_covariance.pose.position.z;
      var xyPair = {
          x: tVal,
          y: zEstimate
      }
      ukfData.push(xyPair);
      // Also plot +/- one standard deviation:
      var heightVariance = message.pose_with_covariance.covariance[14];
      var heightStdDev = Math.sqrt(heightVariance);
      var xyPairStdDevPlus = {
          x: tVal,
          y: zEstimate + heightStdDev
      }
      var xyPairStdDevMinus = {
          x: tVal,
          y: zEstimate - heightStdDev
      }
      ukfPlusSigmaData.push(xyPairStdDevPlus);
      ukfMinusSigmaData.push(xyPairStdDevMinus);
      updateUkfXYChart(message);
      if (!heightChartPaused && !showingUkfAnalysis) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[1].data = ukfData.slice();
          heightChart.data.datasets[2].data = ukfPlusSigmaData.slice();
          heightChart.data.datasets[3].data = ukfMinusSigmaData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
    };

    ukf2dsub.subscribe(ukfCallback);
    ukf7dsub.subscribe(ukfCallback);
    
    ukfStatsSub.subscribe(function(message) {
        currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
        if (!gotFirstHeight) {
            gotFirstHeight = true;
            startTime = currTime;
        }
        tVal = currTime - startTime;
        // Have the plot scroll in time, showing a window of windowSize seconds
        if (tVal > windowSize) {
            spanningFullWindow = true;
            heightChartMinTime = tVal - windowSize;
            heightChartMaxTime = tVal;
            
            // Remove first element of array while difference compared to current
            // time is greater than the windowSize
            while (residualData.length > 0 &&
                   (tVal - residualData[0].x > windowSize)) {
                residualData.splice(0, 1);
                plusSigmaData.splice(0, 1);
                minusSigmaData.splice(0, 1);
            }
        }
        // Add new height error to end of the data array
        // x-y pair
        var zError = message.error;
        var xyPair = {
            x: tVal,
            y: zError
        }
        residualData.push(xyPair);
        // Also plot +/- one standard deviation:
        var heightStdDev = message.stddev;
        var xyPairStdDevPlus = {
            x: tVal,
            y: heightStdDev
        }
        var xyPairStdDevMinus = {
            x: tVal,
            y: -heightStdDev
        }
        plusSigmaData.push(xyPairStdDevPlus);
        minusSigmaData.push(xyPairStdDevMinus);
        if (!heightChartPaused && showingUkfAnalysis) {
            heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
            heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
            heightChart.data.datasets[0].data = residualData.slice();
            heightChart.data.datasets[1].data = plusSigmaData.slice();
            heightChart.data.datasets[2].data = minusSigmaData.slice();
            heightChart.update();
        }
    });
    
    cameraPoseSub.subscribe(function(message) {
        updateCameraPoseXYChart(message);
    });

    function updateGroundTruthXYChart(msg) {
        xPos = msg.pose.position.x;
        yPos = msg.pose.position.y;
        qx = msg.pose.orientation.x;
        qy = msg.pose.orientation.y;
        qz = msg.pose.orientation.z;
        qw = msg.pose.orientation.w;

        if (xPos != null &&
            yPos != null &&
            qx != null &&
            qy != null &&
            qz != null &&
            qw != null) {

            // Quaternion with which to rotate vectors to show the yaw of the
            // drone (and perhaps also the roll and pitch)
            global_to_body_quat = new Quaternion([qw, qx, qy, qz]);
            // v1 = [1, 1, 0];
            // v2 = [1, -1, 0];
            // Drone marker vectors
            v1 = [0.03, 0.03, 0];
            v2 = [0.03, -0.03, 0];
            v3 = [0.0, 0.05, 0];
            rotatedv1 = global_to_body_quat.rotateVector(v1);
            rotatedv2 = global_to_body_quat.rotateVector(v2);
            rotatedv3 = global_to_body_quat.rotateVector(v3);
            xyChart.data.datasets[0].data = [{
                x: (xPos - rotatedv1[0]),
                y: (yPos - rotatedv1[1])
            },
            {
                x: (xPos + rotatedv1[0]),
                y: (yPos + rotatedv1[1])
            }
            ];
            xyChart.data.datasets[1].data = [{
                x: (xPos - rotatedv2[0]),
                y: (yPos - rotatedv2[1])
            },
            {
                x: (xPos + rotatedv2[0]),
                y: (yPos + rotatedv2[1])
            }
            ];
            xyChart.data.datasets[2].data = [{
                x: xPos,
                y: yPos
            },
            {
                x: (xPos + rotatedv3[0]),
                y: (yPos + rotatedv3[1])
            }
            ];
            xyChart.update()
        }
    }

    function updateCameraPoseXYChart(msg) {
        xPos = msg.pose.position.x;
        yPos = msg.pose.position.y;
        qx = msg.pose.orientation.x;
        qy = msg.pose.orientation.y;
        qz = msg.pose.orientation.z;
        qw = msg.pose.orientation.w;

        if (xPos != null &&
            yPos != null &&
            qx != null &&
            qy != null &&
            qz != null &&
            qw != null) {

            // Quaternion with which to rotate vectors to show the yaw of the
            // drone (and perhaps also the roll and pitch)
            global_to_body_quat = new Quaternion([qw, qx, qy, qz]);
            // v1 = [1, 1, 0];
            // v2 = [1, -1, 0];
            // Drone marker vectors
            v1 = [0.03, 0.03, 0];
            v2 = [0.03, -0.03, 0];
            v3 = [0.0, 0.05, 0];
            rotatedv1 = global_to_body_quat.rotateVector(v1);
            rotatedv2 = global_to_body_quat.rotateVector(v2);
            rotatedv3 = global_to_body_quat.rotateVector(v3);
            xyChart.data.datasets[6].data = [{
                x: (xPos - rotatedv1[0]),
                y: (yPos - rotatedv1[1])
            },
            {
                x: (xPos + rotatedv1[0]),
                y: (yPos + rotatedv1[1])
            }
            ];
            xyChart.data.datasets[7].data = [{
                x: (xPos - rotatedv2[0]),
                y: (yPos - rotatedv2[1])
            },
            {
                x: (xPos + rotatedv2[0]),
                y: (yPos + rotatedv2[1])
            }
            ];
            xyChart.data.datasets[8].data = [{
                x: xPos,
                y: yPos
            },
            {
                x: (xPos + rotatedv3[0]),
                y: (yPos + rotatedv3[1])
            }
            ];
            xyChart.update()
        }
    }

    function updateUkfXYChart(msg) {
        xPos = msg.pose_with_covariance.pose.position.x;
        yPos = msg.pose_with_covariance.pose.position.y;
        qx = msg.pose_with_covariance.pose.orientation.x;
        qy = msg.pose_with_covariance.pose.orientation.y;
        qz = msg.pose_with_covariance.pose.orientation.z;
        qw = msg.pose_with_covariance.pose.orientation.w;

        if (xPos != null &&
            yPos != null &&
            qx != null &&
            qy != null &&
            qz != null &&
            qw != null) {

            // Quaternion with which to rotate vectors to show the yaw of the
            // drone (and perhaps also the roll and pitch)
            global_to_body_quat = new Quaternion([qw, qx, qy, qz]);
            // v1 = [1, 1, 0];
            // v2 = [1, -1, 0];
            // Drone marker vectors
            v1 = [0.03, 0.03, 0];
            v2 = [0.03, -0.03, 0];
            v3 = [0.0, 0.05, 0];
            rotatedv1 = global_to_body_quat.rotateVector(v1);
            rotatedv2 = global_to_body_quat.rotateVector(v2);
            rotatedv3 = global_to_body_quat.rotateVector(v3);
            xyChart.data.datasets[3].data = [{
                x: (xPos - rotatedv1[0]),
                y: (yPos - rotatedv1[1])
            },
            {
                x: (xPos + rotatedv1[0]),
                y: (yPos + rotatedv1[1])
            }
            ];
            xyChart.data.datasets[4].data = [{
                x: (xPos - rotatedv2[0]),
                y: (yPos - rotatedv2[1])
            },
            {
                x: (xPos + rotatedv2[0]),
                y: (yPos + rotatedv2[1])
            }
            ];
            xyChart.data.datasets[5].data = [{
                x: xPos,
                y: yPos
            },
            {
                x: (xPos + rotatedv3[0]),
                y: (yPos + rotatedv3[1])
            }
            ];
            xyChart.update()
        }
    }

    emaIrSub.subscribe(function(message) {
      //printProperties(message);
      //console.log("Range: " + message.range);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          // Avoid changing axis limits too often, to avoid shaky plotting?
          // heightChartMinTime = tVal - windowSize;
          // heightChartMaxTime = tVal;

          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (emaData.length > 0 &&
                 (tVal - emaData[0].x > windowSize)) {
              emaData.splice(0, 1);
          }
      }
      // Add new range reading to end of the data array
      // x-y pair
      var xyPair = {
          x: tVal,
          y: message.pose_with_covariance.pose.position.z
      }
      emaData.push(xyPair)
      if (!heightChartPaused && !showingUkfAnalysis) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[4].data = emaData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
    });

    stateGroundTruthSub.subscribe(function(message) {
      //printProperties(message);
      currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
      if (!gotFirstHeight) {
          gotFirstHeight = true;
          startTime = currTime;
      }
      tVal = currTime - startTime;
      // Have the plot scroll in time, showing a window of windowSize seconds
      if (tVal > windowSize) {
          spanningFullWindow = true;
          // Avoid changing axis limits too often, to avoid shaky plotting?
          // heightChartMinTime = tVal - windowSize;
          // heightChartMaxTime = tVal;

          // Remove first element of array while difference compared to current
          // time is greater than the windowSize
          while (stateGroundTruthData.length > 0 &&
                 (tVal - stateGroundTruthData[0].x > windowSize)) {
              stateGroundTruthData.splice(0, 1);
          }
      }
      // Add new height estimate to end of the data array
      // x-y pair
      var zEstimate = message.pose.position.z;
      if (zEstimate != null) {
          var xyPair = {
              x: tVal,
              y: zEstimate
          }
          stateGroundTruthData.push(xyPair);
      }
      updateGroundTruthXYChart(message);
      if (!heightChartPaused && !showingUkfAnalysis) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[5].data = stateGroundTruthData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
    });

    imageStream();
  }

  function imageStream() {
    var image = document.getElementById('cameraImage');
    image.src = "http://" + document.getElementById('hostname').value + ":8080/stream?topic=/pidrone/picamera/image_raw&quality=70";

    var firstImage = document.getElementById('firstImage');
    firstImage.src = "http://" + document.getElementById('hostname').value + ":8080/stream?topic=/pidrone/picamera/first_image&quality=70";

  }

  var irAlphaVal = 0;
  var increasingAlpha = true;
  function pulseIr() {
      // Function to pulse the IR background color in the Height chart when
      // paused, to indicate data are coming in
      if (irAlphaVal <= 0.5 && increasingAlpha) {
          irAlphaVal += 0.005
      } else {
          increasingAlpha = false;
          irAlphaVal -= 0.005
      }
      if (irAlphaVal < 0) {
          increasingAlpha = true;
      }

      // Change gradient depending on whether or not the entire chart window
      // is spanned with data
      if (spanningFullWindow) {
          irBackgroundGradient = ctx.createLinearGradient(300, 0, 600, 0);
      } else {
          irBackgroundGradient = ctx.createLinearGradient(0, 0, 600, 0);
      }

      irBackgroundGradient.addColorStop(0, 'rgba(255, 80, 0, 0)');
      irBackgroundGradient.addColorStop(1, 'rgba(255, 80, 0, '+irAlphaVal.toString()+')');
      heightChart.data.datasets[0].backgroundColor = irBackgroundGradient;
      heightChart.data.datasets[0].fill = true;
      heightChart.update();
  }

/*
 * Key event functions
 */

function publishResetTransform() {
  console.log("reset transform");
  resetpub.publish(emptyMsg);
}

function publishToPosition() {
  console.log("to position");
  positionMsg.data = true;
  positionPub.publish(positionMsg);
}

function publishToVelocity() {
  console.log("to velocity");
  positionMsg.data = false;
  positionPub.publish(positionMsg);
}

function publishToggleMap() {
    console.log("toggle map")
    mappub.publish(emptyMsg);
}

function publishArm() {
  console.log("arm");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    velocityControlPub.publish(twistMsg)
  }
  modeMsg.mode = "ARMED"
  modepub.publish(modeMsg);
}

function publishDisarm() {
  console.log("disarm");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
  modeMsg.mode = "DISARMED"
  modepub.publish(modeMsg);
}

function publishTakeoff() {
  console.log("takeoff");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
  modeMsg.mode = "FLYING"
  modepub.publish(modeMsg);
}

function publishTranslateLeft() {
  console.log("translate left");
  if (positionMsg.data == true) {
    poseMsg.position.x = -0.1
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = -0.1
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateRight() {
  console.log("translate right");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0.1
    poseMsg.position.y = 0
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0.1
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateForward() {
  console.log("translate forward");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = 0.1
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0.1
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateBackward() {
  console.log("translate backward");
  if (positionMsg.data == true) {
    poseMsg.position.x = 0
    poseMsg.position.y = -0.1
    poseMsg.position.z = 0
    positionControlPub.publish(poseMsg)
  } else {
    twistMsg.linear.x = 0
    twistMsg.linear.y = -0.1
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
  }
}

function publishTranslateUp() {
  console.log("translate up");
  poseMsg.position.x = 0
  poseMsg.position.y = 0
  poseMsg.position.z = 0.05
  positionControlPub.publish(poseMsg)
}

function publishTranslateDown() {
  console.log("translate down");
  poseMsg.position.x = 0
  poseMsg.position.y = 0
  poseMsg.position.z = -0.05
  positionControlPub.publish(poseMsg)
}

function publishYawLeft() {
    console.log("yaw left")
    modeMsg.mode = "FLYING"
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = -50
    velocityControlPub.publish(twistMsg)
}

function publishYawRight() {
    console.log("yaw right")
    modeMsg.mode = "FLYING"
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 50
    velocityControlPub.publish(twistMsg)
}

function publishZeroVelocity() {
  console.log("zero velocity");
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.z = 0
    velocityControlPub.publish(twistMsg)
}

/*
 * Handle IR chart and UKF map
 */


var rawIrDataset = {
  label: 'Raw IR Readings',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(255, 80, 0, 0.8)',
  backgroundColor: 'rgba(255, 80, 0, 0)',
  lineTension: 0, // remove smoothing
  itemID: 0
};
var rawIrData = Array(0);

var ukfDataset = {
  label: 'UKF Filtered Height',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(49, 26, 140, 0.8)',
  backgroundColor: 'rgba(49, 26, 140, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 1
}
var ukfData = Array(0);

var ukfPlusSigmaDataset = {
  label: 'UKF +sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 0,
  pointRadius: 0,
  fill: '+1', // fill to the next dataset
  borderColor: 'rgba(49, 26, 140, 0)', // full transparency
  backgroundColor: 'rgba(49, 26, 140, 0.1)',
  lineTension: 0, // remove smoothing
  itemID: 2
}
var ukfPlusSigmaData = Array(0);

var ukfMinusSigmaDataset = {
  label: 'UKF -sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 0,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(49, 26, 140, 0)', // full transparency
  //backgroundColor: 'rgba(49, 26, 140, 0.1)'
  lineTension: 0, // remove smoothing
  itemID: 3
}
var ukfMinusSigmaData = Array(0);

var emaDataset = {
  label: 'EMA-Smoothed Altitude',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(252, 70, 173, 0.8)',
  backgroundColor: 'rgba(252, 70, 173, 0)',
  lineTension: 0, // remove smoothing
  itemID: 4
}
var emaData = Array(0);

var stateGroundTruthDataset = {
  label: 'Ground Truth Height',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(0, 0, 0, 0.8)',
  backgroundColor: 'rgba(0, 0, 0, 0)',
  lineTension: 0, // remove smoothing
  itemID: 5
}
var stateGroundTruthData = Array(0);

//--------------------------------------

var residualDataset = {
  label: 'Error between UKF and Ground Truth',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(209, 81, 58, 0.8)',
  backgroundColor: 'rgba(209, 81, 58, 0)',
  lineTension: 0, // remove smoothing
}
var residualData = Array(0);

var plusSigmaDataset = {
  label: '+1 sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: '+1', // fill to the next dataset
  borderColor: 'rgba(49, 26, 140, 0.8)',
  backgroundColor: 'rgba(49, 26, 140, 0.1)',
  lineTension: 0, // remove smoothing
}
var plusSigmaData = Array(0);

var minusSigmaDataset = {
  label: '-1 sigma',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(49, 26, 140, 0.8)',
  backgroundColor: 'rgba(49, 26, 140, 0.1)',
  lineTension: 0, // remove smoothing
}
var minusSigmaData = Array(0);


var ctx;
var xyctx;

function loadHeightChartStandardView() {
    ctx = document.getElementById("heightChart").getContext('2d');
    heightChart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                rawIrDataset,
                ukfDataset,
                ukfPlusSigmaDataset,
                ukfMinusSigmaDataset,
                emaDataset,
                stateGroundTruthDataset
            ]
        },
        options: {
            animation: {
               duration: 0,
            },
            scales: {
                yAxes: [{
                    ticks: {
                        beginAtZero: true,
                        min: 0,
                        max: 0.6,
                        stepSize: 0.1
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'Height (meters)'
                    }
                }],
                xAxes: [{
                    type: 'linear',
                    display: false,
                    ticks: {
                        min: 0,
                        max: windowSize,
                        stepSize: windowSize
                    }
                }]
            },
            legend: {
              display: true,
              labels: {
                  // Filter out UKF standard deviation datasets and datasets
                  // that have no data in them
                  filter: function(itemInLegend, chartData) {
                      var itemIndex = itemInLegend.datasetIndex;
                      return ((itemIndex != ukfPlusSigmaDataset.itemID &&
                               itemIndex != ukfMinusSigmaDataset.itemID) &&
                               (chartData.datasets[itemIndex].data.length != 0));
                  }
              }
            },
        }
    });
}

function loadHeightChartUkfAnalysis() {
    ctx = document.getElementById("heightChart").getContext('2d');
    heightChart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                residualDataset,
                plusSigmaDataset,
                minusSigmaDataset
            ]
        },
        options: {
            animation: {
               duration: 0,
            },
            scales: {
                yAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: 'Height (meters)'
                    }
                }],
                xAxes: [{
                    type: 'linear',
                    display: false,
                    ticks: {
                        min: 0,
                        max: windowSize,
                        stepSize: windowSize
                    }
                }]
            },
            legend: {
              display: true
            },
        }
    });
}

$(document).ready(function() {
    loadHeightChartStandardView();
    xyctx = document.getElementById("xyChart").getContext('2d');
    xyChart = new Chart(xyctx, {
        type: 'line',
        data: {
            datasets: [
                {
                  data: Array(0), // initialize array of length 0
                  borderWidth: 1.5,
                  pointRadius: 0,
                  fill: false,
                  borderColor: 'rgba(0, 0, 0, 1)',
                  backgroundColor: 'rgba(0, 0, 0, 0)',
                  lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(0, 0, 0, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(0, 0, 0, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(49, 26, 140, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(49, 26, 140, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(49, 26, 140, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(255, 80, 0, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(255, 80, 0, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
              {
                data: Array(0), // initialize array of length 0
                borderWidth: 1.5,
                pointRadius: 0,
                fill: false,
                borderColor: 'rgba(255, 80, 0, 1)',
                backgroundColor: 'rgba(0, 0, 0, 0)',
                lineTension: 0, // remove smoothing
              },
            ]
        },
        options: {
            animation: {
               duration: 0,
            },
            scales: {
                yAxes: [{
                    ticks: {
                        min: -1,
                        max: 1,
                        stepSize: 0.1
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'y position (meters)'
                    }
                }],
                xAxes: [{
                    type: 'linear',
                    ticks: {
                        min: -400.0/250.0,
                        // max: 1,
                        max: 400.0/250.0,
                        stepSize: 0.1,
                        display: true
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'x position (meters)'
                    }
                }]
            },
            legend: {
              display: false
            },
        }
    });

    init();
});

$(window).on("beforeunload", function(e) {
    closeSession();
});

function changeHeightChartYScaleMin() {
    heightChart.options.scales.yAxes[0].ticks.min = parseFloat(document.getElementById('heightMin').value);
    heightChart.update();
}

function changeHeightChartYScaleMax() {
    heightChart.options.scales.yAxes[0].ticks.max = parseFloat(document.getElementById('heightMax').value);
    heightChart.update();
}

function togglePauseHeightChart(btn) {
    heightChartPaused = !heightChartPaused;
    if (heightChartPaused) {
        btn.value = 'Play'
        irAlphaVal = 0;
    } else {
        btn.value = 'Pause'
        heightChart.data.datasets[0].backgroundColor = 'rgba(255, 80, 0, 0)';
        heightChart.data.datasets[0].fill = false;
    }
}

function toggleUkfAnalysis(btn) {
    showingUkfAnalysis = !showingUkfAnalysis;
    if (showingUkfAnalysis) {
        btn.value = 'Standard View'
        heightChart.destroy();
        loadHeightChartUkfAnalysis();
    } else {
        btn.value = 'UKF Analysis'
        heightChart.destroy();
        loadHeightChartStandardView();
    }
}

function togglePauseXYChart(btn) {
    // TODO: Implement this function
    console.log('Pause button pressed')
}

function publishVelocityMode() {
    positionMsg.data = false;
    positionPub.publish(positionMsg)
}

function publishPositionMode() {
    positionMsg.data = true;
    positionPub.publish(positionMsg)
}

function setControls () {
    x = document.getElementById("controlX").value;
    y = document.getElementById("controlY").value;
    z = document.getElementById("controlZ").value;

    if (positionMsg.data == true) {
        poseMsg.position.x = Number(parseFloat(x));
        poseMsg.position.y = Number(parseFloat(y));
        poseMsg.position.z = Number(parseFloat(z));
        positionControlPub.publish(poseMsg);
    } else {
        twistMsg.linear.x = Number(parseFloat(x));
        twistMsg.linear.y = Number(parseFloat(y));
        twistMsg.linear.z = Number(parseFloat(z));
        velocityControlPub.publish(twistMsg);
    }
}

/*
 * Listen for key events
*/

$(document).keyup(function(event){
  var char = String.fromCharCode(event.which || event.keyCode);
  if (char == "J" || char == "L" || char == "K" || char == "I" || char == "W" || char == "S" || char == "A" || char == "D") {
    publishZeroVelocity();
  }
});

$(document).keypress(function(event){
  var char = String.fromCharCode(event.which || event.keyCode);
  if (char == ';') {
    publishArm();
  } else if (char == ' ') {
    publishDisarm();
  } else if (char == 'r') {
    publishResetTransform();
  } else if (char == 't') {
    publishTakeoff();
  } else if (char == 'p') {
    publishToPosition();
  } else if (char == 'v') {
    publishToVelocity();
  } else if (char == 'm') {
    publishToggleMap();
  }
});

$(document).keydown(function(event){
  var char = String.fromCharCode(event.which || event.keyCode);
  // console.log("Key down: " + char);
  if (char == 'J') {
    publishTranslateLeft();
  } else if (char == 'L') {
    publishTranslateRight();
  } else if (char == "K") {
    publishTranslateBackward();
  } else if (char == "I") {
    publishTranslateForward();
  } else if (char == "W") {
    publishTranslateUp();
  } else if (char == "S") {
    publishTranslateDown();
  } else if (char == "A") {
    publishYawLeft();
  } else if (char == "D") {
    publishYawRight();
  } else {
    //console.log('undefined key: ' + event.keyCode);
  }
});
