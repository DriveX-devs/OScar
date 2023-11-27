/**
 * This file contains the client JavaScript logic, in charge of receiving the coordinates of the objects
 * (i.e. the vehicles) and their heading from the server (which in turn receives them from the S-LDM via UDP).
 * This script uses the received information in order to render a Leaflet JS map showing all the moving nodes.
 * The server->client communication is realized thanks to socket.io.
 */

// Constant for unavailable coordinates values
const VIS_LONLAT_INVALID = -80000;

// Constant for an unavailable heading value (VIS_HEADING_INVALID)
const VIS_HEADING_INVALID = 361;

// Car and circle icon indeces
const CAR_ICO_IDX = 0;
const CIRCLE_ICO_IDX = 1;
const GREEN_CIRCLE_ICO_IDX = 2;

// Start socket.io()
const socket = io();

var map_rx = false;

// Markers array
var markers = [];
// Markers icon array ('0' for carIcon, i.e. CAR_ICO_IDX, '1' for circleIcon, i.e. CIRCLE_ICO_IDX)
var markersicons = [];

// Map reference variable
var leafletmap = null;

// Create a new icon for the "car" markers
var icon_length = 102;
var icon_height = 277;
var icon_length_circle = 144;
var icon_height_circle = 143;
var icon_length_circle_green = 94;
var icon_height_circle_green = 94;
var icon_scale_factor = 11;
var carIcon = L.icon({
	iconUrl: './img/triangle.png',

	iconSize:     [icon_length/icon_scale_factor, icon_height/icon_scale_factor], // size of the icon
	iconAnchor:   [icon_length/(icon_scale_factor*2), icon_height/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
	popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});

var circleIcon = L.icon({
	iconUrl: './img/circle.png',

	iconSize:     [icon_length_circle/icon_scale_factor, icon_height_circle/icon_scale_factor], // size of the icon
	iconAnchor:   [icon_length_circle/(icon_scale_factor*2), icon_height_circle/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
	popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});

var greenCircleIcon = L.icon({
	iconUrl: './img/green_circle.png',

	iconSize:     [icon_length_circle_green/icon_scale_factor, icon_height_circle_green/icon_scale_factor], // size of the icon
	iconAnchor:   [icon_length_circle_green/(icon_scale_factor*2), icon_height_circle_green/(icon_scale_factor*2)], // point of the icon which will correspond to marker's location
	popupAnchor:  [0, 0] // point from which the popup should open relative to the iconAnchor
});


// Receive the first message from the server
socket.on('message', (msg) => {
	if (msg == null) {
		document.getElementById('statusid').innerHTML = '<p>Waiting for a connection from the S-LDM</p>';
	} else {
		let msg_fields = msg.split(",");

		switch (msg_fields[0]) {
			// "map areas draw" message: "map_areas,<lat>,<lon>,<minlat>,<minlon>,<maxlat>,<maxlon>,<lat_ext_factor>,<lon_ext_factor>,<mapbox token>"
			case 'map_areas':
				if (map_rx === false) {
					if (msg_fields.length !== 10) {
						console.error("VehicleVisualizer: Error: received a corrupted map draw message from the server (map_areas type)");
					} else {
						console.info("VehicleVisualizer: The map will be drawn centered at: ", msg_fields[1], msg_fields[2]);
						console.info("VehicleVisualizer: displayed area: [",msg_fields[3],",",msg_fields[4],"],[",msg_fields[5],",",msg_fields[6],"]");
						document.getElementById('statusid').innerHTML = '';
						let mapbox_token;

						console.log(msg_fields[3]);

						if(msg_fields[9] != "none") {
							mapbox_token = msg_fields[9];
						} else {
							mapbox_token = null;
						}
						leafletmap = draw_map(parseFloat(msg_fields[1]), parseFloat(msg_fields[2]), parseFloat(msg_fields[3]), 
							parseFloat(msg_fields[4]), parseFloat(msg_fields[5]), parseFloat(msg_fields[6]), 
							parseFloat(msg_fields[7]), parseFloat(msg_fields[8]), mapbox_token);
						map_rx = true;
					}
				}
				break;

			// "map draw" message: "map,<lat>,<lon>,<mapbox token>"
			case 'map':
				if (map_rx === false) {
					if (msg_fields.length !== 4) {
						console.error("VehicleVisualizer: Error: received a corrupted map draw message from the server");
					} else {
						console.info("VehicleVisualizer: The map will be drawn centered at: ", msg_fields[1], msg_fields[2]);
						document.getElementById('statusid').innerHTML = '';
						let mapbox_token;

						console.log(msg_fields[3]);

						if(msg_fields[3] != "none") {
							mapbox_token = msg_fields[3];
						} else {
							mapbox_token = null;
						}
						leafletmap = draw_map(parseFloat(msg_fields[1]), parseFloat(msg_fields[2]), VIS_LONLAT_INVALID, 
							VIS_LONLAT_INVALID, VIS_LONLAT_INVALID, VIS_LONLAT_INVALID, 
							VIS_LONLAT_INVALID, VIS_LONLAT_INVALID, mapbox_token);
						map_rx = true;
					}
				}
				break;
			// "object update"/vehicle update message: "object,<unique object ID>,<lat>,<lon>,<stationtype>,<heading>"
			case 'object':
				if(msg_fields.length !== 6) {
					console.error("VehicleVisualizer: Error: received a corrupted object update message from the server.");
				} else {
					update_marker(leafletmap,msg_fields[1],parseFloat(msg_fields[2]),parseFloat(msg_fields[3]),parseInt(msg_fields[4]),parseFloat(msg_fields[5]));
				}
				break;
			// "object clean"/vehicle removal message: "objclean,<unique object ID>"
			case 'objclean':
				if(msg_fields.length !== 2) {
					console.error("VehicleVisualizer: Error: received a corrupted object clean message from the server.");
				} else {
					let id = parseInt(msg_fields[1]);
					if(id in markers) {
						markers[id].remove();
						delete markers[id];
					}
				}
				break;
			// This 'case' is added just for additional safety. As the server is shut down every time a "terminate" message
			// is received from the S-LDM and no "terminate" message is forwarded via socket.io, this point should never be
			// reached
			case 'terminate':
				console.log("The server has been terminated.");
				break;

			default:
				console.warn("VehicleVisualizer: Warning: unknown message type received from the server");
		}
	}
});

// This function is used to update the position (and heading/rotation) of a marker/moving object on the map
function update_marker(mapref,id,lat,lon,stationtype,heading)
{
	if(mapref == null) {
		console.error("VehicleVisualizer: null map reference when attempting to update an object")
	} else {
		// If the object ID has never been seen before, create a new marker
		if(!(id in markers)) {
			let newmarker;
			let initial_icon;
			let initial_icon_idx;

			if(stationtype === 0) {
				initial_icon = greenCircleIcon;
				initial_icon_idx = GREEN_CIRCLE_ICO_IDX;
			} else {
				// Set a circular icon when the heading is not available, otherwise use the regular carIcon (i.e. for the time being, a triangle)
				if(heading >= VIS_HEADING_INVALID) {
					initial_icon = circleIcon;
					initial_icon_idx = CIRCLE_ICO_IDX;
				} else {
					initial_icon = carIcon;
					initial_icon_idx = CAR_ICO_IDX;
				}
			}

			// Attempt to use an icon marker
			newmarker = L.marker([lat,lon], {icon: initial_icon}).addTo(mapref);
			newmarker.setRotationAngle(heading);

			// Old circle marker (no more used)
			// newmarker = L.circleMarker([lat,lon],{radius: 8, fillColor: "#c48612", color: "#000000", weight: 1, opacity: 1, fillOpacity: 0.8}).addTo(mapref);
			// Set the initial popup value (if the heading is invalid/unavailable, do not specify any value in degrees
			if(heading < VIS_HEADING_INVALID) {
				newmarker.bindPopup("ID: "+id+" - Heading: "+heading+" deg");
			} else {
				newmarker.bindPopup("ID: "+id+" - Heading: unavailable");
			}
			markers[id]=newmarker;
			markersicons[id]=initial_icon_idx;
		// If the object ID has already been seen before, just update its attributes (i.e. position, rotation angle and popup content)
		} else {
			let marker = markers[id];
			marker.setLatLng([lat,lon]);
			marker.setRotationAngle(heading);
			// Update the popup content if the heading value becomes invalid/unavailable after being available
			// or if the heading value if actually available (to specify the most up-to-date heading value)
			if(heading >= VIS_HEADING_INVALID && markersicons[id] === CAR_ICO_IDX) {
				marker.setPopupContent("ID: "+id+" - Heading: unavailable");
			} else if(heading < VIS_HEADING_INVALID) {
				marker.setPopupContent("ID: "+id+" - Heading: "+heading+" deg");
			}

			if(stationtype !== 0) {
				// If the heading becomes unavailable or invalid (but it was not before), change the icon of the vehicle to a circle
				if(heading >= VIS_HEADING_INVALID && (markersicons[id] === CAR_ICO_IDX || markersicons[id] === GREEN_CIRCLE_ICO_IDX)) {
					marker.setIcon(circleIcon);
					markersicons[id] = CIRCLE_ICO_IDX;
				// If the heading becomes available after being unavailable, change the icon of the vehicle to the "car" icon
				} else if(heading < VIS_HEADING_INVALID && (markersicons[id] === CIRCLE_ICO_IDX || markersicons[id] === GREEN_CIRCLE_ICO_IDX)) {
					marker.setIcon(carIcon);
					markersicons[id] = CAR_ICO_IDX;
				}
			} else {
				if(markersicons[id] === CAR_ICO_IDX || markersicons[id] === CIRCLE_ICO_IDX) {
					marker.setIcon(greenCircleIcon);
					markersicons[id] = GREEN_CIRCLE_ICO_IDX;
				}
			}
		}
	}
}

// This function is used to draw the whole map at the beginning, on which vehicles will be placed
// It expects as arguments the lat and lon value where the map should be centered
function draw_map(lat,lon,minlat,minlon,maxlat,maxlon,lat_ext_factor,lon_ext_factor,mapbox_token) {
	let standardlayer;

	// If no Mapbox token is specified, create a basic view layer based on OpenStreetMap (occasional use only! Heavy usage is forbidden!)
	if(mapbox_token == null) {
		standardlayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors | <a href="https://www.openstreetmap.org/fixthemap">Report a problem in the map</a>',
			id: 'mapbox/streets-v11',
			// tileSize: 512,
			// zoomOffset: -1,
		});
	}

	// Create standard (street view), hybrid and satellite layers based on Mapbox
	// They are normally not enabled as you need an access token to use them
	let hybridlayer, satellitelayer;

	if(mapbox_token != null) {
		hybridlayer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=' + mapbox_token, {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.mapbox.com/about/maps/">Mapbox</a> | S-LDM vehicle visualizer hybrid view',
			id: 'mapbox/satellite-streets-v11',
			tileSize: 512,
			zoomOffset: -1,
		});

		hybridlayer.setOpacity(0.7);

		satellitelayer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=' + mapbox_token, {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.mapbox.com/about/maps/">Mapbox</a> | S-LDM vehicle visualizer satellite view',
			id: 'mapbox/satellite-v9',
			tileSize: 512,
			zoomOffset: -1,
		});

		satellitelayer.setOpacity(0.7);

		standardlayer = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=' + mapbox_token, {
			maxZoom: 30,
			attribution: '&copy; <a href="https://www.mapbox.com/about/maps/">Mapbox</a> | S-LDM vehicle visualizer streets view',
			id: 'mapbox/streets-v11',
			tileSize: 512,
			zoomOffset: -1,
		});
	}

	// Main map object creation (default layer: standardlayer, i.e. OpenStreetMap or Mapbox street view)
	var mymap = L.map('mapid', {
		center: [lat, lon],
		zoom: 17,
		layers: standardlayer
	});

	// Add all the layers to the map, adding a control button to dynamically change the current map layer, if more than one layer can be used
	if (mapbox_token != null) {
		let basemaps = {
			"Street view": standardlayer,
			"Hybrid view": hybridlayer,
			"Satellite view": satellitelayer,
		};

		L.control.layers(basemaps).addTo(mymap);

		console.log("A Mapbox token has been specified. Multiple layers will be available.")
	}

	if(minlat!==VIS_LONLAT_INVALID && minlon!==VIS_LONLAT_INVALID && maxlat!==VIS_LONLAT_INVALID && maxlon!==VIS_LONLAT_INVALID) {
		L.rectangle([[minlat,minlon],[maxlat,maxlon]], {color: 'red', fill: false}).addTo(mymap);

		if(lat_ext_factor!==VIS_LONLAT_INVALID && lon_ext_factor!==VIS_LONLAT_INVALID) {
			L.rectangle([[minlat-lat_ext_factor,minlon-lon_ext_factor],[maxlat+lat_ext_factor,maxlon+lon_ext_factor]], {color: 'green', fill: false}).addTo(mymap);
		}
	}

	// Print on the console that the map has been successfully rendered
	console.log("VehicleVisualizer: Map Created!")

	return mymap;
}
