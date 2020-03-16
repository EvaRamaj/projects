
import React from 'react';
import {Map, InfoWindow, Marker, GoogleApiWrapper} from 'google-maps-react';
import Geocoder from 'react-native-geocoding';

export class MapContainer extends React.Component {

    constructor(props) {
        super(props);
        console.log('here:', props);
        Geocoder.init('AIzaSyAyesbQMyKVVbBgKVi2g6VX7mop2z96jBo'); // use a valid API key

        Geocoder.from("Colosseum")
            .then(json => {
                let location = json.results[0].geometry.location;
                console.log(location);
            })
            .catch(error => console.warn(error));

        this.defaultProps = {
            zoom: 14,
            initialCenter: {
                lat: 40.297424,
                lng: 21.795655
            }
        };
        // const {lat, lng} = this.defaultProps.initialCenter;

        this.state = {
            showingInfoWindow: false,
            activeMarker: {},
            selectedPlace: {
                name: ''
            },
            initialCenter:{
                lat: props.coord.lat,
                lng: props.coord.lng
            }
            // currentLocation: {
            //     lat: lat,
            //     lng: lng
            // }

        };

        this.onMarkerClick = this.onMarkerClick.bind(this);
        this.onMapClicked = this.onMapClicked.bind(this);
    }




    onMarkerClick(props, marker, e) {
        console.log(props,marker);
        this.setState({
            selectedPlace: props,
            activeMarker: marker,
            showingInfoWindow: true
        });
    }


    onMapClicked(props){
        console.log(props.google);
        if (this.state.showingInfoWindow) {
            this.setState({
                showingInfoWindow: false,
                activeMarker: null
            });
        }
    };

    // loadMap() {
    //     if (this.props && this.props.google) {
    //         // google is available
    //         const {google} = this.props;
    //         const maps = google.maps;
    //
    //         const mapRef = this.refs.map;
    //         const node = ReactDOM.findDOMNode(mapRef);
    //
    //         let {initialCenter, zoom} = this.defaultProps;
    //         const {lat, lng} = initialCenter;
    //         const center = new maps.LatLng(lat, lng);
    //         const mapConfig = Object.assign({}, {
    //             center: center,
    //             zoom: zoom
    //         });
    //         this.map = new maps.Map(node, mapConfig);
    //     }
    // }

    render() {
        let self = this;
        const style = {
            width: '285px',
            height: '200px'
        };

        let points = [
            { lat: 40.287424, lng: 21.795655 },
            { lat: 40.307424, lng: 21.795655 },
            { lat: 40.277424, lng: 21.795655 },
        ];

        return (

            <Map google={this.props.google}
                 style={style}
                 initialCenter={this.state.initialCenter}
                 zoom={this.defaultProps.zoom}
                 onClick={this.onMapClicked}>
                <Marker onClick={this.onMarkerClick}
                        name={'Pistolaki'} />

                {/*{Object.keys(points).map(function(key) {*/}
                    {/*console.log('key: ', key);*/}
                    {/*return(<Marker*/}
                        {/*onClick={self.onMarkerClick}*/}
                        {/*title={'The marker`s title will appear as a tooltip.'}*/}
                        {/*name={key}*/}
                        {/*position={points[key]} />*/}
                    {/*)*/}
                {/*})}*/}

                <InfoWindow
                    marker={this.state.activeMarker}
                    onOpen={this.windowHasOpened}
                    onClose={this.windowHasClosed}
                    visible={this.state.showingInfoWindow}>
                    <div>
                        <h1>{this.state.selectedPlace.name}</h1>
                    </div>
                </InfoWindow>

            </Map>
        )
    }
}



export default GoogleApiWrapper({
    apiKey: 'AIzaSyAyesbQMyKVVbBgKVi2g6VX7mop2z96jBo',
})(MapContainer)