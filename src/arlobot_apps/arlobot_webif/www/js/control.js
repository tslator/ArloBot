function layoutControls() {

    var motorStick = $('#motor_stick');
    var panTiltStick = $('#pan_tilt_stick');
    var cameraImage = $('#camera');
    var btnShutdown = $( "#btnShutdown" );

    console.log( screen.width + " " + screen.height );

    if ( screen.width > screen.height )
    {
        // In landscape mode
        motorStick.css( 'left', 0.1*screen.width - motorStick.width()/2 );
        motorStick.css( 'top', 0.5*screen.height - motorStick.height()/2 );
        panTiltStick.css( 'left', 0.9*screen.width - panTiltStick.width()/2 );
        panTiltStick.css( 'top', 0.5*screen.height - panTiltStick.height()/2 );
        btnShutdown.css( 'position', 'fixed' );
        btnShutdown.css( 'left', 0.9*screen.width - btnShutdown.width()/2 );
        btnShutdown.css( 'top', 0.1*screen.height );
    }
    else
    {
        // In portrait mode
        motorStick.css( 'left', 0.2*screen.width - motorStick.width()/2 );
        motorStick.css( 'top', 0.7*screen.height - motorStick.height()/2 );
        panTiltStick.css( 'left', 0.8*screen.width - panTiltStick.width()/2 );
        panTiltStick.css( 'top', 0.7*screen.height - panTiltStick.height()/2 );
        btnShutdown.css( 'position', 'fixed' );
        btnShutdown.css( 'left', 0.5*screen.width - btnShutdown.width()/2 );
        btnShutdown.css( 'top', 0.6*screen.height - btnShutdown.height()/2 );
    }
    
    cameraImage.css( 'max-width', screen.width );
}

$( window ).on( "orientationchange", function() {
    setTimeout( layoutControls, 300 ); } );

$(document).ready(function() {

     webSocketPort = window.location.port;
     if ( webSocketPort == "" )
     {
        webSocketPort = "80";
     }
     //cameraPort = ( 8000 + parseInt( webSocketPort ) ).toString();
     
     webSocketURL = "http://" + window.location.hostname + ":" + webSocketPort + "/robot_control";
     //cameraURL = "http://" + window.location.hostname  + ":" + cameraPort + "/?action=stream";
     cameraURL = "http://" + window.location.hostname  + ":" + webSocketPort + "/camera";
     socket = new SockJS( webSocketURL );
     var camera_socket = new SockJS( cameraURL );
     
     camera_socket.onopen = function(){
         console.log('open');
     };
     camera_socket.onmessage = function(e){
        console.log('message', e.data);
     }
     camera_socket.onclose = function(){
        console.log('close');
     }
     
     var shutdownComplete = false;
 
     $("#camera").attr( "src", cameraURL );

     $('#motor_stick').joystick({
         xSnap: true,
         ySnap: true,
         moveEvent: function(pos) {
            console.log("Move " + pos.x + " " + pos.y);
            if ( socket.readyState == SockJS.OPEN ) {
                socket.send( "Move " + pos.x + " " + pos.y ); } },
         endEvent: function(pos) { 
            if ( socket.readyState == SockJS.OPEN ) {
                socket.send( "Move " + pos.x + " " + pos.y ); } },
         updateIntervalMS: 500,
         updateEvent: function(pos) { 
            if ( socket.readyState == SockJS.OPEN
                && ( 0.0 != pos.x || 0.0 != pos.y ) ) {
                socket.send( "Move " + pos.x + " " + pos.y ); } }
     });
     $('#pan_tilt_stick').joystick({
         xSnap: true,
         ySnap: true,
         moveEvent: function(pos) { 
            if ( socket.readyState == SockJS.OPEN ) {
                socket.send( "PanTilt " + pos.x + " " + pos.y ); } },
         endEvent: function(pos) { 
            if ( socket.readyState == SockJS.OPEN ) {
                socket.send( "PanTilt " + pos.x + " " + pos.y ); } },
         clickEvent: function(clickPos) { 
            if ( socket.readyState == SockJS.OPEN 
                && Math.abs( clickPos.x ) < 0.3 && Math.abs( clickPos.y ) < 0.3 ) 
            {
                socket.send( "Centre" ); 
            } },
         updateIntervalMS: 500,
         updateEvent: function(pos) { 
            if ( socket.readyState == SockJS.OPEN 
                && ( 0.0 != pos.x || 0.0 != pos.y ) ) {
                socket.send( "PanTilt " + pos.x + " " + pos.y ); } }
     });
     $('#motor_stick').joystick( 'value', 0.0, 0.0 );
     $('#pan_tilt_stick').joystick( 'value', 0.0, 0.0 );
     
     $( "#btnShutdown" ).click( function() {
     
        var SHUTDOWN_TIME_MS = 25000;
     
        if ( socket.readyState == SockJS.OPEN ) 
        {
            socket.send( "Shutdown" );
        }
        
        $( "#shutdownProgress" ).removeAttr('style');
        $( "#shutdownText" ).text( "Shutting Down" );
        
        var modalShutdownDialog = $( "#modalShutdownDialog" );
        modalShutdownDialog.css( "opacity", 1 );
        modalShutdownDialog.css( "top", "10%" );
        modalShutdownDialog.css( "visibility", "visible" );
        
        // Wait for the Pi to shutdown. The wait time is determined experimentally with an extra couple of seconds
        // added on for safety
        setTimeout( function() {
            $( "#shutdownProgress" ).css( "visibility", "hidden" );
            $( "#shutdownText" ).text( "It is now safe to switch off the power to the Pi" );
            shutdownComplete = true;
        }, SHUTDOWN_TIME_MS );
        
      } );
    
    // Add a periodic function to keep the camera and socket alive
    setInterval( function() { 
        if ( socket.readyState == SockJS.OPEN ) {
            //socket.send( "StartStreaming" );
            //$("#camera").attr( "src", cameraURL );
            
            if ( shutdownComplete )
            {
                $( "#modalShutdownDialog" ).removeAttr('style');    // Reset the shutdown dialog by removing the style attribute added by jQuery
                shutdownComplete = false;
            }
            
        } else {
            socket = new SockJS( webSocketURL );
        } }, 1000 );
    
    layoutControls();
}); 
