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
  function init() {
    // Connect to ROS.
    var url = 'ws://' + document.getElementById('hostname').value + ':9090'
    ros = new ROSLIB.Ros({
      url : url
    });

    function closeSession(){
      console.log("Closing connections.");
      ros.disconnect();
      return false;
    }
    window.onbeforeunload = closeSession;

  ros.on('error', function(error) {
      console.log('ROS Master:  Error, check console.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML='<p>Error detected; check console.</p>';
  });

  ros.on('connection', function() {
      console.log('ROS Master:  Connected.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML="<p style='color:green'>Connected!</p>";
  });

  ros.on('close', function() {
      console.log('ROS Master:  Connection closed.');
      //printProperties(error);
      document.getElementById('statusMessage').innerHTML="<p style='color:red'>Connection closed.</p>";
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

    statesub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/state',
      messageType : 'pidrone_pkg/State',
      queue_length : 2,
      throttle_rate : 2
    });
    statesub.subscribe(function(message) {
      //printProperties(message);
      var mynumber = myround(message.vbat, 2);
      document.getElementById('vbat').innerHTML=mynumber
      if (message.vbat <= 11.3) {
        document.getElementById('vbat').innerHTML=mynumber + " EMPTY!";
      } else {
        document.getElementById('vbat').innerHTML=mynumber;
      }

    });


    irsub = new ROSLIB.Topic({
      ros : ros,
      name : '/pidrone/infrared',
      messageType : 'sensor_msgs/Range',
      queue_length : 2,
      throttle_rate : 5
    });
    irsub.subscribe(function(message) {
      //printProperties(message);
      //console.log("Range: " + message.range);
      irChart.data.datasets[0].data[count] = message.range;
      if (count - 1 < 0) {
        irChart.data.datasets[1].data[windowSize - 1] = 0;
      } else{
        irChart.data.datasets[1].data[count - 1] = 0;
      }
      irChart.data.datasets[1].data[count] = 60;
      count = count + 1;
      count = count % irChart.data.datasets[0].data.length;
      irChart.update();
      //console.log("Data: " + irChart.data.datasets[0].data);

    });

  
    var imu = document.getElementById("imu");
    empty(imu);

    // Create the main viewer.
    var imuviewer = new ROS3D.Viewer({
      divID : 'imu',
      width : 320,
      height : 240,
      antialias : true
    });


    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 5.0,
      fixedFrame : '/base'
    });

    // Setup the marker client.
    markerClient = new ROS3D.MarkerClient({
      ros : ros,
      tfClient : tfClient,
      topic : document.getElementById('imutopic').value,
      rate : 5.0,
      rootObject : imuviewer.scene
    });


    // Create the main viewer.
//    var imageviewer = new MJPEGCANVAS.Viewer({
//      divID : 'camera',
//      host : '192.168.42.1',
//      width : 320,
//      height : 240,
//      topic : '/pidrone/picamera/image_raw'
//    });



    imageStream();
  }

  function imageStream() {
    var image = document.getElementById('cameraImage');
    image.src = "http://" + document.getElementById('hostname').value + ":8080/stream?topic=/pidrone/picamera/image_raw&quality=70";

    var firstImage = document.getElementById('firstImage');
    firstImage.src = "http://" + document.getElementById('hostname').value + ":8080/stream?topic=/pidrone/picamera/first_image&quality=70";

  }

  function markerUpdate() {
    markerClient.topicName = document.getElementById('imutopic').value;
    markerClient.subscribe();
  }

$(document).keypress(function(event){
  alert(String.fromCharCode(event.which));
});
