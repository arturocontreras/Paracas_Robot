
var ge;
google.load("earth", "1");
var placemark1, placemark2;
var map = null;
var contador = 0;
var Latitudes = [];
var Longitudes = [];
var flag_ruta = 1;
var contador_puntos = 0;
var multGeoPlacemark = null;

function init() {
    google.earth.createInstance('map3d', initCB, failureCB);
    //google.earth.addEventListener('map3d', 'click', function (event) {
    //    alert('Lat: ' + event.latLng.lat() + ' Lng: ' + event.latLng.lng());
    //});

}

function initCB(instance) {
    ge = instance;
    ge.getWindow().setVisibility(true);
    ge.getLayerRoot().enableLayerById(ge.LAYER_BORDERS, true);

    LoadKml();

    var lookAt = ge.createLookAt('');
    //lookAt.setLatitude(-13.894086);
    //lookAt.setLongitude(-76.125432);
    lookAt.setLatitude(-12.025204);
    lookAt.setLongitude(-77.047082);

    //motores
    // -12.025204
    // -77.047082

    //Cancha2
    //-12.024913
    //-77.047443

    //Cancha1
    //-12.024642
    //-77.047725

    lookAt.setRange(100.0);
    ge.getView().setAbstractView(lookAt);

    // Create the placemark.
    placemark1 = ge.createPlacemark('');
    placemark1.setName("P1");


    // Define a custom icon.
    /*var icon = ge.createIcon('');
    icon.setHref('http://maps.google.com/mapfiles/kml/paddle/red-circle.png');
    var style = ge.createStyle('');
    style.getIconStyle().setIcon(icon);
    style.getIconStyle().setScale(5.0);
    placemark.setStyleSelector(style);*/

    // Set the placemark's location.  
    var point = ge.createPoint('');
    point.setLatitude(-12.025204);
    point.setLongitude(-77.047082);
    //-12.024642
    //-77.047725
    placemark1.setGeometry(point);

    // Add the placemark to Earth.
    ge.getFeatures().appendChild(placemark1);

    //graficar();

    eventos();
    google.earth.addEventListener(ge.getWindow(), 'mousedown', function (event) {

        //alert('Evento');
        //alert('Lat: ' + event.getLatitude() + ' Lng: ' + event.getLongitude());


        if (event.getButton() == 2) {

            window.external.coordenadas(event.getLatitude(), event.getLongitude());

            if (flag_ruta == 1) {
                contador++;
                Latitudes[contador] = event.getLatitude();
                Longitudes[contador] = event.getLongitude();
                //crear_ruta();

                var x = document.getElementById("mode").checked;
                var xPi, yPi, xPf, yPf;
                if (x == true) {
                    window.external.ruteo(event.getLatitude(), event.getLongitude());

                    if (contador == 1) {
                        xPi = event.getLongitude();
                        yPi = event.getLatitude();
                        window.external.ruteo(event.getLatitude(), event.getLongitude());

                    }

                    if (contador == 2) {
                        xPi = event.getLongitude();
                        yPi = event.getLatitude();
                        window.external.ruteo(event.getLatitude(), event.getLongitude());

                    }

                }

                else {

                    if (contador >= 2) {
                        var line1 = ge.createLineString('');
                        line1.getCoordinates().pushLatLngAlt(Latitudes[contador - 1], Longitudes[contador - 1], 0);
                        line1.getCoordinates().pushLatLngAlt(Latitudes[contador], Longitudes[contador], 0);
                        line1.setTessellate(true);
                        line1.setAltitudeMode(ge.ALTITUDE_CLAMP_TO_GROUND);

                        var multiGeometry = ge.createMultiGeometry('');
                        multiGeometry.getGeometries().appendChild(line1);

                        multGeoPlacemark = ge.createPlacemark('');
                        //var multGeoPlacemark = ge.createPlacemark('');
                        multGeoPlacemark.setGeometry(multiGeometry);

                        multGeoPlacemark.setStyleSelector(ge.createStyle(''));
                        var lineStyle = multGeoPlacemark.getStyleSelector().getLineStyle();
                        lineStyle.setWidth(1);
                        lineStyle.getColor().set('ffff0000');

                        ge.getFeatures().appendChild(multGeoPlacemark);

                        multGeoPlacemark.setName('rutas');
                    }

                }
            }
        }
    }
   )


    LoadKml();
}

function crear_ruta() {
    window.external.ruteo(event.getLatitude(), event.getLongitude());

    var x = document.getElementById("mode").checked;
    var xPi, yPi, xPf, yPf;
    if (x == true)
    {
        window.external.ruteo(event.getLatitude(), event.getLongitude());

        if (contador == 1) {
            xPi = event.getLongitude();
            yPi = event.getLatitude();
            window.external.ruteo(event.getLatitude(), event.getLongitude());
            
        }

        if (contador == 2) {
            xPi = event.getLongitude();
            yPi = event.getLatitude();
            window.external.ruteo(event.getLatitude(), event.getLongitude());

        }
        
    }

    else
    {

        if (contador >= 2) {
            var line1 = ge.createLineString('');
            line1.getCoordinates().pushLatLngAlt(Latitudes[contador - 1], Longitudes[contador - 1], 0);
            line1.getCoordinates().pushLatLngAlt(Latitudes[contador], Longitudes[contador], 0);
            line1.setTessellate(true);
            line1.setAltitudeMode(ge.ALTITUDE_CLAMP_TO_GROUND);

            var multiGeometry = ge.createMultiGeometry('');
            multiGeometry.getGeometries().appendChild(line1);

            var multGeoPlacemark = ge.createPlacemark('');
            multGeoPlacemark.setGeometry(multiGeometry);

            multGeoPlacemark.setStyleSelector(ge.createStyle(''));
            var lineStyle = multGeoPlacemark.getStyleSelector().getLineStyle();
            lineStyle.setWidth(1);
            lineStyle.getColor().set('ffff0000');

            ge.getFeatures().appendChild(multGeoPlacemark);

            multGeoPlacemark.setName('rutas');
        }

    }

}

function graficar() {
    var line1 = ge.createLineString('');
    line1.getCoordinates().pushLatLngAlt(-13.895951, -76.125909, 0);
    line1.getCoordinates().pushLatLngAlt(-13.894086, -76.125432, 0);
    line1.setTessellate(true);
    line1.setAltitudeMode(ge.ALTITUDE_CLAMP_TO_GROUND);

    var multiGeometry = ge.createMultiGeometry('');
    multiGeometry.getGeometries().appendChild(line1);

    var multGeoPlacemark = ge.createPlacemark('');
    multGeoPlacemark.setGeometry(multiGeometry);

    multGeoPlacemark.setStyleSelector(ge.createStyle(''));
    var lineStyle = multGeoPlacemark.getStyleSelector().getLineStyle();
    lineStyle.setWidth(1);
    lineStyle.getColor().set('ffff0000');

    ge.getFeatures().appendChild(multGeoPlacemark);

    multGeoPlacemark.setName('ruta');

}

function graficar2() {
    var line3 = ge.createLineString('');
    line3.getCoordinates().pushLatLngAlt(-13.894086, -76.125432, 0);
    line3.getCoordinates().pushLatLngAlt(-13.894544, -76.123455, 0);

    line3.setTessellate(true);
    line3.setAltitudeMode(ge.ALTITUDE_CLAMP_TO_GROUND);

    var multiGeometry = ge.createMultiGeometry('');
    multiGeometry.getGeometries().appendChild(line3);

    var multGeoPlacemark = ge.createPlacemark('');
    multGeoPlacemark.setGeometry(multiGeometry);

    multGeoPlacemark.setStyleSelector(ge.createStyle(''));
    var lineStyle = multGeoPlacemark.getStyleSelector().getLineStyle();
    lineStyle.setWidth(1);
    lineStyle.getColor().set('ffff0000');

    ge.getFeatures().appendChild(multGeoPlacemark);

    multGeoPlacemark.setName('ruta');

}

function eventos() {

    google.earth.addEventListener(placemark1, 'click', doSomething);

}

function doSomething() {

    // Create the placemark.
    placemark2 = ge.createPlacemark('');
    placemark2.setName("P3");


    // Set the placemark's location.  
    var point = ge.createPoint('');
    point.setLatitude(-13.894544);
    point.setLongitude(-76.123455);
    placemark2.setGeometry(point);

    // Add the placemark to Earth.
    ge.getFeatures().appendChild(placemark2);

    window.external.activacion();


}

function failureCB(errorCode) {
}

function LoadKml() {
    /*
    var link = ge.createLink('');
    var href = 'https://sites.google.com/site/archivosjbc02/archivos/DemoTacna.kml'
    link.setHref(href);

    var networkLink = ge.createNetworkLink('');
    networkLink.set(link, true, true); // Sets the link, refreshVisibility, and flyToView.
    ge.getFeatures().appendChild(networkLink);
    */

    var href = 'https://sites.google.com/site/acm66contreras/ge/sembrios.kml'
    google.earth.fetchKml(ge, href, function (kmlObject) {
        if (kmlObject)
            ge.getFeatures().appendChild(kmlObject);
    });


}


function showJavascriptHelloWorld() {
    alert("Hello world");
    // Create the placemark.
    placemark3 = ge.createPlacemark('');
    placemark3.setName("P7");

    // Set the placemark's location.  
    var point = ge.createPoint('');
    point.setLatitude(-13.894086);
    point.setLongitude(-76.125433);
    placemark3.setGeometry(point);

    // Add the placemark to Earth.
    ge.getFeatures().appendChild(placemark3);

    graficar2();
}

function mostrarcoordenada(lat_in, lon_in) {

    contador_puntos++;

    if (contador_puntos > 1) {
        //  alert('removido latitud :' + lat_in + '   longitud :' + lon_in);
        ge.getFeatures().removeChild(punto_actual); //Borrando el placemark anterior
    }
    // Set the placemark's location.  
    var point_actual = ge.createPoint('');
    point_actual.setLatitude(lat_in);
    point_actual.setLongitude(lon_in);
    punto_actual = ge.createPlacemark('');
    punto_actual.setGeometry(point_actual);
    punto_actual.setName("R");
    // Add the placemark to Earth.
    ge.getFeatures().appendChild(punto_actual);

    var lookAt = ge.createLookAt('');
    lookAt.setLatitude(lat_in);
    lookAt.setLongitude(lon_in);
    lookAt.setRange(80.0);
    ge.getView().setAbstractView(lookAt);

    //Centro Cancha FIM 1
    //Lat:-12.024642
    //Lon:-77.047725

}

function dibujarlimites(lat_in, lat_fin, lon_in, lon_fin) {
    //alert('latitud :' + lat_in + '   longitud :' + lat_fin);

    var linex = ge.createLineString('');
    linex.getCoordinates().pushLatLngAlt(lat_in, lon_in, 0);
    linex.getCoordinates().pushLatLngAlt(lat_fin, lon_fin, 0);
    linex.setTessellate(true);
    linex.setAltitudeMode(ge.ALTITUDE_CLAMP_TO_GROUND);

    var multiGeometry = ge.createMultiGeometry('');
    multiGeometry.getGeometries().appendChild(linex);

    var multGeoPlacemark = ge.createPlacemark('');
    multGeoPlacemark.setGeometry(multiGeometry);

    multGeoPlacemark.setStyleSelector(ge.createStyle(''));
    var lineStyle = multGeoPlacemark.getStyleSelector().getLineStyle();
    lineStyle.setWidth(1);
    lineStyle.getColor().set('ff0000ff');

    ge.getFeatures().appendChild(multGeoPlacemark);

    multGeoPlacemark.setName('limites');
}

function dibujarcorners(centerLat, centerLng) {
    //alert('latitud :' + lat_in + '   longitud :' + lat_fin);

    var polygonPlacemark = ge.createPlacemark('');
    polygonPlacemark.setGeometry(ge.createPolygon(''));
    var outer = ge.createLinearRing('');
    polygonPlacemark.getGeometry().setOuterBoundary(makeCircle(centerLat, centerLng, 0.000005));
    polygonPlacemark.setName('placemark');
    ge.getFeatures().appendChild(polygonPlacemark);
}

function Px_to_corner(lat_in, lat_fin, lon_in, lon_fin) {
    //alert('latitud :' + lat_in + '   longitud :' + lat_fin);

    var linex = ge.createLineString('');
    linex.getCoordinates().pushLatLngAlt(lat_in, lon_in, 0);
    linex.getCoordinates().pushLatLngAlt(lat_fin, lon_fin, 0);
    linex.setTessellate(true);
    linex.setAltitudeMode(ge.ALTITUDE_CLAMP_TO_GROUND);

    var multiGeometry = ge.createMultiGeometry('');
    multiGeometry.getGeometries().appendChild(linex);

    var multGeoPlacemark = ge.createPlacemark('');
    multGeoPlacemark.setGeometry(multiGeometry);

    multGeoPlacemark.setStyleSelector(ge.createStyle(''));
    var lineStyle = multGeoPlacemark.getStyleSelector().getLineStyle();
    lineStyle.setWidth(1);
    lineStyle.getColor().set('kk0000ff');

    ge.getFeatures().appendChild(multGeoPlacemark);

    multGeoPlacemark.setName('busqueda_cercano');
}



function wp_permitido(flag) {

    if (flag == 0) {
        flag_ruta = 0;
    }
    else flag_ruta = 1;
}



function makeCircle(centerLat, centerLng, radius) {

    var ring = ge.createLinearRing('');
    var steps = 25;
    var pi2 = Math.PI * 2;

    for (var i = 0; i < steps; i++) {
        var lat = parseFloat(centerLat) + radius * Math.cos(i / steps * pi2);
        var lng = parseFloat(centerLng) + radius * Math.sin(i / steps * pi2);
        ring.getCoordinates().pushLatLngAlt(lat, lng, 0);
    }
    return ring;
}

function borrar_coordenadas() {
    //Borrar contador. las coordenadas 

    for (var i = 0; i < contador; i++) {
        Latitudes[i] = 0;
        Longitudes[i] = 0;
    }

    contador = 0;
    
    //Borrando lineas grabadas
    //ge.getFeatures().removeChild(multGeoPlacemark);

    var children = ge.getFeatures().getChildNodes();
    for(var i = 0; i < children.getLength(); i++) { 
        var child = children.item(i);
        if (child.getType() == 'KmlPlacemark') {

            //if(child.getId()=='path'){
            //    ge.getFeatures().removeChild(child);
            // }
            ge.getFeatures().removeChild(child);
        }
    }
    
}


google.setOnLoadCallback(init);