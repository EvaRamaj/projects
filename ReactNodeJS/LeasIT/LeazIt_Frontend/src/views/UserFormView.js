"use strict";

import React from 'react';

import UserForm from './../components/UserForm';

import UserService from '../services/UserService';
import AuthService from "../services/AuthService";
import Geocoder from "react-native-geocoding";


export class UserFormView extends React.Component {

    constructor(props) {
        super(props);
        Geocoder.init('AIzaSyAyesbQMyKVVbBgKVi2g6VX7mop2z96jBo');

    }

    componentWillMount(){
        if(this.props.location.state != undefined && this.props.location.state.user != undefined) {
            this.setState({
                loading: false,
                user: this.props.location.state.user,
                error: undefined
            });
        }
        else {
            this.setState({
                loading: true,
                error: undefined
            });

            UserService.getMyProfile().then((data) => {
                this.setState({
                    user: data,
                    loading: false,
                    error: undefined
                });
            }).catch((e) => {
                console.error(e);
            });
        }
    }

    updateUser(user) {

        Geocoder.from(user.address)

            .then(json => {

                let location = json.results[0].geometry.location;
                console.log(location);
                user.coordinates = {
                    lat: location.lat,
                    lng: location.lng
                };
                console.log('user:', user.coordinates);
                UserService.updateUser(user).then((data) => {
                    this.props.history.goBack();
                }).catch((e) => {
                    console.error(e);
                    this.setState(Object.assign({}, this.state, {error: 'Error while creating user'}));
                });
            })
            .catch(error => console.warn(error));



    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (<UserForm user={this.state.user} onSubmit={(user) => this.updateUser(user)} error={this.state.error} />);
    }
}