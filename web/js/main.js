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
var heartbeatPub;
var heightChart;
var windowSize;
var gotFirstHeight = false;
var startTime;
var heightChartPaused = false;
var spanningFullWindow = false;

function closeSession(){
  console.log("Closing connections.");
  ros.disconnect();
  return false;
}

function init() {
    // Connect to ROS.
    var url = 'ws://' + document.getElementById('hostname').value + ':9090'
    ros = new ROSLIB.Ros({
        url : url
    });

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

    modepub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/set_mode',
      messageType : 'pidrone_pkg/Mode'
    });

    modeMsg = new ROSLIB.Message({
      mode: 0,
     });

    emptyMsg = new ROSLIB.Message({
     });


    heartbeatPub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/heartbeat',
      messageType : 'std_msgs/String'
    });
    heartbeatpubmsg = new ROSLIB.Message({data: "Javascript API"})

    setInterval(function(){
      heartbeatPub.publish(heartbeatpubmsg);
      //console.log("heartbeat");
    }, 1000);

    resetpub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/reset_transform',
      messageType : 'std_msgs/Empty'
    });

    togglepub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/toggle_transform',
      messageType : 'std_msgs/Empty'
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

    irsub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/infrared',
      messageType : 'sensor_msgs/Range',
      queue_length : 2,
      throttle_rate : 80
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
      if (!heightChartPaused) {
          heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[0].data = rawIrData.slice();
          heightChart.update();
      } else {
          pulseIr();
      }
      //console.log("Data: " + heightChart.data.datasets[0].data);
      //console.log('tVal: ' + tVal)
    });
    
    // imusub = new ROSLIB.Topic({
    //   ros : ros,
    //   name : '/pidrone/imu_throttle',
    //   messageType : 'sensor_msgs/Imu',
    //   queue_length : 2,
    //   throttle_rate : 50
    // });
    // imusub.subscribe(function(message) {
    //   //printProperties(message);
    //   //console.log("Range: " + message.range);
    //   currTime = message.header.stamp.secs + message.header.stamp.nsecs/1.0e9;
    //   if (!gotFirstHeight) {
    //       gotFirstHeight = true;
    //       startTime = currTime;
    //   }
    //   tVal = currTime - startTime;
    //   // Have the plot scroll in time, showing a window of windowSize seconds
    //   if (tVal > windowSize) {
    //       spanningFullWindow = true;
    //       heightChartMinTime = tVal - windowSize;
    //       heightChartMaxTime = tVal;
    //       // Remove first element of array while difference compared to current
    //       // time is greater than the windowSize
    //       while (rawImuData.length > 0 &&
    //              (tVal - rawImuData[0].x > windowSize)) {
    //           rawImuData.splice(0, 1);
    //       }
    //   }
    //   // Add new z acceleration reading to end of the data array
    //   // x-y pair
    //   var xyPair = {
    //       x: tVal,
    //       y: message.linear_acceleration.z
    //   }
    //   rawImuData.push(xyPair)
    //   if (!heightChartPaused) {
    //       heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
    //       heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
    //       heightChart.data.datasets[6].data = rawImuData.slice();
    //       heightChart.update();
    //   } else {
    //       pulseIr();
    //   }
    //   //console.log("Data: " + heightChart.data.datasets[0].data);
    //   //console.log('tVal: ' + tVal)
    // });
    
    statesub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state',
        messageType : 'pidrone_pkg/State',
        queue_length : 2,
        throttle_rate : 80
    });
    statesub.subscribe(function(message) {
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
      updateXYChart(message);
      if (!heightChartPaused) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[1].data = ukfData.slice();
          heightChart.data.datasets[2].data = ukfPlusSigmaData.slice();
          heightChart.data.datasets[3].data = ukfMinusSigmaData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
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
    
    function updateXYChart(msg) {
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
            console.log('xpos')
            console.log(xPos)
            console.log('ypos')
            console.log(yPos)
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
    
    emaIrSub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/state_ema',
      messageType : 'pidrone_pkg/State',
      queue_length : 2,
      throttle_rate : 80
    });
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
      if (!heightChartPaused) {
          // heightChart.options.scales.xAxes[0].ticks.min = heightChartMinTime;
          // heightChart.options.scales.xAxes[0].ticks.max = heightChartMaxTime;
          heightChart.data.datasets[4].data = emaData.slice();
          // Avoid updating too often, to avoid shaky plotting?
          // heightChart.update();
      }
    });
    
    stateGroundTruthSub = new ROSLIB.Topic({
        ros : ros,
        name : '/pidrone/state_ground_truth',
        messageType : 'pidrone_pkg/StateGroundTruth',
        queue_length : 2,
        throttle_rate : 80
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
      if (!heightChartPaused) {
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

function publishArm() {
  console.log("arm");
  modeMsg.mode = 0
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}
function publishResetTransform() {
  console.log("reset transform");
  resetpub.publish(emptyMsg);
}

function publishToggleTransform() {
  console.log("toggle transform");
  togglepub.publish(emptyMsg);
}


function publishDisarm() {
  console.log("disarm");
  modeMsg.mode = 4
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}

function publishZeroVelocity() {
  console.log("zero velocity");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}


function publishTakeoff() {
  console.log("takeoff");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}


function publishTranslateLeft() {
  console.log("translate left");
  modeMsg.mode = 5
  modeMsg.x_velocity = -10
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}

function publishTranslateRight() {
  console.log("translate right");
  modeMsg.mode = 5
  modeMsg.x_velocity = 10
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}

function publishTranslateForward() {
  console.log("translate forward");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 10
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);

}

function publishTranslateBackward() {
  console.log("translate backward");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = -10
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}


function publishTranslateUp() {
  console.log("translate up");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0.05
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}

function publishTranslateDown() {
  console.log("translate down");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = -0.05
  modeMsg.yaw_velocity = 0
  modepub.publish(modeMsg);
}


function publishYawLeft() {
  console.log("yaw left");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = -50
  modepub.publish(modeMsg);
}

function publishYawRight() {
  console.log("yaw right");
  modeMsg.mode = 5
  modeMsg.x_velocity = 0
  modeMsg.y_velocity = 0
  modeMsg.z_velocity = 0
  modeMsg.yaw_velocity = 50
  modepub.publish(modeMsg);
}

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

var rawImuDataset = {
  label: 'Raw IMU Z Acceleration (m/s^2)',
  data: Array(0), // initialize array of length 0
  borderWidth: 1.5,
  pointRadius: 0,
  fill: false,
  borderColor: 'rgba(80, 255, 0, 0.8)',
  backgroundColor: 'rgba(80, 255, 0, 0)',
  lineTension: 0, // remove smoothing
  itemID: 6
};
var rawImuData = Array(0);


var ctx;
var xyctx;
$(document).ready(function() {
    ctx = document.getElementById("heightChart").getContext('2d');
    windowSize = 5;
    heightChart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                rawIrDataset,
                ukfDataset,
                ukfPlusSigmaDataset,
                ukfMinusSigmaDataset,
                emaDataset,
                stateGroundTruthDataset,
                rawImuDataset
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

function togglePauseXYChart(btn) {
    // TODO: Implement this function
    console.log('Pause button pressed')
}

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
  } else if (char == 'h') {
    publishZeroVelocity();
  } else if (char == 'r') {
    publishResetTransform();
  } else if (char == 't') {
    publishTakeoff();
  } else if (char == 'p') {
    publishToggleTransform();
  }
});
