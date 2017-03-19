
var ge;
google.load("earth", "1");
var contador_puntos = 0;
var punto_actual = null;

function init() {
    google.earth.createInstance('map3d', initCB, failureCB);

}

function initCB(instance) {
    ge = instance;
    ge.getWindow().setVisibility(true);
    ge.getLayerRoot().enableLayerById(ge.LAYER_BORDERS, true);

    var lookAt = ge.createLookAt('');
    lookAt.setLatitude(-12.025204);
    lookAt.setLongitude(-77.047082);
    lookAt.setRange(100.0);
    ge.getView().setAbstractView(lookAt);

    // Create the placemark.
    placemark1 = ge.createPlacemark('');
    placemark1.setName("P1");

    // Set the placemark's location.  
    var point = ge.createPoint('');
    point.setLatitude(-12.025204);
    point.setLongitude(-77.047082);
    placemark1.setGeometry(point);

    // Add the placemark to Earth.
    ge.getFeatures().appendChild(placemark1);

    eventos();

    LoadKml();
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

    var href = 'https://sites.google.com/site/archivosjbc02/archivos/DemoTacna.kml'
    google.earth.fetchKml(ge, href, function (kmlObject) {
        if (kmlObject)
            ge.getFeatures().appendChild(kmlObject);
    });


}


function mostrarcoordenada(lat_in, lon_in) {

    //contador_puntos++;

    //if (contador_puntos > 1) {
        //  alert('removido latitud :' + lat_in + '   longitud :' + lon_in);
        //ge.getFeatures().removeChild(punto_actual); //Borrando el placemark anterior
    //}
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
    lookAt.setRange(50.0);
    ge.getView().setAbstractView(lookAt);

    //Centro Cancha FIM 1
    //Lat:-12.024642
    //Lon:-77.047725

}

function borrar_coordenadas() {

    var children = ge.getFeatures().getChildNodes();
    for (var i = 0; i < children.getLength() ; i++) {
        var child = children.item(i);
        if (child.getType() == 'KmlPlacemark') {
            ge.getFeatures().removeChild(child);
        }
    }

}


google.setOnLoadCallback(init);