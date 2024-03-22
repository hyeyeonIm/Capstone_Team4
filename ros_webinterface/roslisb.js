$(document).ready(function() {
    // ROS 연결 초기화
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    // ROS 연결 상태 업데이트
    var statusSpan = document.getElementById("status");
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


    // 페이지 로드 시 매핑 상태 확인
    if(localStorage.getItem("mappingFinished") === "true") {
        $(".Start_Map, .Stop_Map").hide();
        $(".Mapping_Again").show();
    } else {
        $(".Start_Map, .Stop_Map").show();
        $(".Mapping_Again").hide();
    }

    // 매핑 시작 버튼 클릭 이벤트
    $(".Start_Map").click(async function() {
        try {
            const data = await $.get("http://localhost:8080/start_hector");
            alert(`Response: ${data}\nStatus: success`);
        } catch (error) {
            console.error("An error occurred:", error);
            alert(`Error starting mapping: ${error.statusText}`);
        }
    });

    // 매핑 중지 및 저장 버튼 클릭 이벤트
    $('.Stop_Map').click(async function() {   
        var mapName = prompt("지도의 이름을 입력해주세요:", "defaultMapName");
        if (mapName) {
            try {
                const saveResponse = await $.get(`http://localhost:8080/stop_and_save/${mapName}`);
                alert(saveResponse); // 저장 완료 알림
    
                const loadResponse = await $.get(`http://localhost:8080/load_map/${mapName}`);
                console.log(loadResponse); // 콘솔에 로딩 결과를 출력
                alert(loadResponse); // 로딩 완료 알림
            } catch (error) {
                console.error("An error occurred:", error);
                alert(`Error: ${error.statusText}`);
            }
        }
        // 매핑 상태 저장
        localStorage.setItem("mappingFinished", "true");
        $(".Start_Map").hide();
        $(".Stop_Map").hide();
        $(".Mapping_Again").show();
    });

    // 다시 매핑하기 버튼 클릭 이벤트
    $(".Mapping_Again").click(function() {
        // 매핑 상태 초기화 및 저장
        localStorage.setItem("mappingFinished", "true");
        $(".Start_Map").show();
        $(".Stop_Map").show();
        $(this).hide(); // "다시 매핑하기" 버튼 숨기기
    });

    // 지도 시각화 관련 코드는 그대로 유지
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
});



    // // Example of subscribing to a topic
    // var moveBaseFeedbackListener = new ROSLIB.Topic({
    //     ros: ros,
    //     name: '/move_base/feedback',
    //     messageType: 'move_base_msgs/MoveBaseActionFeedback'
    // });

    // moveBaseFeedbackListener.subscribe(function(message) {
    //     console.log('Received move_base feedback: ', message);
    //     // Handle or visualize the feedback
    // });

    // Additional functionalities as needed