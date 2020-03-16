"use strict";

import React from 'react';

import UserSignup from '../components/UserSignup';

import AuthService from '../services/AuthService';
import Geocoder from "react-native-geocoding";


export class UserSignupView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {};
        Geocoder.init('AIzaSyAyesbQMyKVVbBgKVi2g6VX7mop2z96jBo');

    }

    signup(user) {
        Geocoder.from(user.address)
            .then(json => {
                let location = json.results[0].geometry.location;
                user.coordinates = {
                    lat: location.lat,
                    lng: location.lng
                };
                AuthService.register(user).then((data) => {
                    this.props.history.push('/');
                }).catch((e) => {
                    console.error(e);
                    this.setState({
                        error: e
                    });
                })
            })
            .catch(error => console.warn(error));


    }

    render() {
        return (
            <UserSignup onSubmit={(user) => this.signup(user)} error={this.state.error}></UserSignup>
        );
    }
}