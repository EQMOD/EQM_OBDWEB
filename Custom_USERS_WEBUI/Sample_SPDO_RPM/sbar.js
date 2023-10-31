var speedTop = 0;
var speedAvg = 0;
var engineSpeedTop = 0;
var totalMoveCount = 0;
var speedSumTotal = 0;
var speedAnimation = 1;

$(document).ready(function() {
    // websocket
    // --------------------------------------------------------------------------
  function retrievedata(action) {
        var spDOWebsocket = new WebSocket("ws://192.168.0.10/ws");
        spDOWebsocket.onmessage = function(event) {
            var res = event.data.split("#");
			var currentSpeedUse = $.trim(res[0]);	
			speedCurrent = Math.ceil(parseInt(currentSpeedUse) * 1.0 );
			var currentrpmUse = $.trim(res[1]);
			rpmCurrentInt = Math.ceil(parseInt(currentrpmUse) * 1.0 );			
			rpmCurrent = Math.ceil(parseInt(currentrpmUse) * 0.019375 );  // Bar max at 155/8000 rpm 


                    updaterpmBar(rpmCurrent);
					$('.txtSpeed').text(speedCurrent);
					$('.txtRpm').text(rpmCurrentInt);

        };
        spDOWebsocket.onopen = function() {
            spDOWebsocket.send(action);
        };
        spDOWebsocket.onerror = function(e) {
            console.log("err: " + e.toString());
        };
    }
 

  function updaterpmBar(rpmval) {
       for (var i = 150; i >= 105; i -= 5) {
        var barClassName = '.rpmBar_' + i;
        if (rpmval >= i) {
          switch (i) {
            case 150:
              backgroundColor = '#FF0000';
              break;
            case 145:
              backgroundColor = '#FF0000';
              break;
            case 140:
              backgroundColor = '#FF0000';
              break;
            case 135:
              backgroundColor = '#FF0000';
              break;
            case 130:
              backgroundColor = '#FF0000';
              break;
            case 125:
              backgroundColor = '#FE2E2E';
              break;
            case 120:
              backgroundColor = '#FF451C';
              break;
            case 115:
              backgroundColor = '#FF6932';
              break;
            case 110:
              backgroundColor = '#FE9A2E';
              break;
            case 105:
              backgroundColor = '#FECC20';
              break;
          }
          $(barClassName).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName).css({ 'background-color': 'transparent' });
        }
      }
      for (var j = 100; j >= 55; j -= 5) {
        var barClassName2 = '.rpmBar_' + j;
        if (rpmval >= j) {
          switch (j) {
            case 100:
              backgroundColor = '#FFED2E';
              break;
            case 95:
              backgroundColor = '#FFF430';
              break;
            case 90:
              backgroundColor = '#F7FE2E';
              break;
            case 85:
              backgroundColor = '#C8FE2E';
              break;
            case 80:
              backgroundColor = '#9AFE2E';
              break;
            case 75:
              backgroundColor = '#64FE2E';
              break;
            case 70:
              backgroundColor = '#2EFE2E';
              break;
            case 65:
              backgroundColor = '#2EFE64';
              break;
            case 60:
              backgroundColor = '#2EFE9A';
              break;
            case 55:
              backgroundColor = '#58FAD0';
              break;
          }
          $(barClassName2).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName2).css({ 'background-color': 'transparent' });
        }
      }
      for (var k = 50; k >= 5; k -= 5) {
        var barClassName3 = '.rpmBar_' + k;
        if (rpmval >= k) {
          switch (k) {
            case 50:
              backgroundColor = '#58FAD0';
              break;
            case 45:
              backgroundColor = '#58FAD0';
              break;
            case 40:
              backgroundColor = '#58FAD0';
              break;
            case 35:
              backgroundColor = '#58FAD0';
              break;
            case 30:
              backgroundColor = '#58FAD0';
              break;
            case 25:
              backgroundColor = '#81F7D8';
              break;
            case 20:
              backgroundColor = '#A9F5E1';
              break;
            case 15:
              backgroundColor = '#CEF6EC';
              break;
            case 10:
              backgroundColor = '#E0F8F1';
              break;
            case 5:
              backgroundColor = '#EFFBF8';
              break;
          }
          $(barClassName3).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName3).css({ 'background-color': 'transparent' });
        }
      }
  }


    // Start data retrieval
    setTimeout(function() {
    retrievedata('speed_rpm200');
    }, 2000);
});






