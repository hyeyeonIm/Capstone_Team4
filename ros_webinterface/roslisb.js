$(document).ready(function() {
    // 매핑 시작 버튼 클릭 이벤트
    $(".Start_Map").click(function() {
        // 서버에 매핑 시작 요청 보내기
        $.get("http://localhost:8080/start_hector", function(data, status) {
            alert("Response: " + data + "\nStatus: " + status);
        });
    });

    // 매핑 중지 및 저장 버튼 클릭 이벤트
    $('.Stop_Map').click(function() {
        var mapName = prompt("지도의 이름을 입력해주세요:", "defaultMapName");
        if (mapName) {
            $.get(`http://localhost:8080/stop_and_save/${mapName}`, function(data) {
                alert(data);
            });
        }
        $(".Start_Map").hide();
        $(".Stop_Map").hide();
        $(".Mapping_Again").show();
    });

    $(".Mapping_Again").click(function() {
        // 다시 매핑하기 버튼 로직
        $(".Start_Map").show();
        $(".Stop_Map").show();
        $(this).hide(); // "다시 매핑하기" 버튼 숨기기
    });

    // ROS 및 지도 시각화 관련 코드는 그대로 유지
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
        width: 500,
        height: 500
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
    var moveBaseFeedbackListener = new ROSLIB.Topic({
        ros: ros,
        name: '/move_base/feedback',
        messageType: 'move_base_msgs/MoveBaseActionFeedback'
    });

    moveBaseFeedbackListener.subscribe(function(message) {
        console.log('Received move_base feedback: ', message);
        // Handle or visualize the feedback
    });

    // Additional functionalities as needed
});
