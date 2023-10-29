

$(document).ready(function() {
    // websocket
    // --------------------------------------------------------------------------
    function retrievedata(action) {
        var batMonWebsocket = new WebSocket("ws://192.168.0.10/ws");
        batMonWebsocket.onmessage = function(event) {
            var res = event.data.split("#");

                    updateBatmon(res[0]);

        };
        batMonWebsocket.onopen = function() {
            batMonWebsocket.send(action);
        };
        batMonWebsocket.onerror = function(e) {
            console.log("err: " + e.toString());
        };
    }
    // --------------------------------------------------------------------------
    // websocket end
    // --------------------------------------------------------------------------
    //
    // BEGINN TPMS UPDATES
    //
	// --------------------------------------------------------------------------
	function updateBatmon(value) {
		value = parseFloat(value); // parseInt?
		$('.BatMonVAL').text(value+" Volts");
	}



    // Start data retrieval
    setTimeout(function() {
    retrievedata('bm2_1000');
    }, 2000);
});

// TBD: Swap Tire Positions? See Speedometer
// SaveSpeedBarLayout();
// No need to save layout, only save order of IDs




