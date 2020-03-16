"use strict";

import React from 'react';

import UserLogin from '../components/UserLogin';

import AuthService from '../services/AuthService';
import Geocoder from "react-native-geocoding";


export class UserLoginView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {};

    }

    login(user) {
        console.log(user);
        AuthService.login(user.username, user.password).then((data) => {
            if(data.role === 'Admin'){
                this.props.history.push('/admin_dashboard');
            }
            else{
                this.props.history.push('/user_dashboard');
            }
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
    }

    render() {
        return (
          <UserLogin onSubmit={(user) => this.login(user)} error={this.state.error}></UserLogin>
        );
    }
}