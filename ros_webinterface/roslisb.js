// Roslib.js - Updated for Real-time Map Visualization

document.addEventListener('DOMContentLoaded', function() {
    // Initialize ROS connection
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });
  
    // Connection status elements
    var statusSpan = document.getElementById("status");
  
    // Update connection status
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        statusSpan.innerHTML = "Connected";
    });
  
    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        statusSpan.innerHTML = "Error";
    });
  
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        statusSpan.innerHTML = "Closed";
    });
  
    // Map visualization
    var viewer = new ROS2D.Viewer({
        divID: 'map',
        width: 700,
        height: 700
    });
  
    var gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        topic: '/map',
        continuous: true
    });
  
    gridClient.on('change', function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
  
    // Example of subscribing to a topic
    // This example subscribes to /move_base/feedback for demonstration purposes
    var moveBaseFeedbackListener = new ROSLIB.Topic({
        ros: ros,
        name: '/move_base/feedback',
        messageType: 'move_base_msgs/MoveBaseActionFeedback'
    });
  
    moveBaseFeedbackListener.subscribe(function(message) {
        console.log('Received move_base feedback: ', message);
        // Here you can implement how to handle or visualize the feedback
    });
  
    // Implement additional functionalities as needed
  });
  