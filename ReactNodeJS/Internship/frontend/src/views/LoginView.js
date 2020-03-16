"use strict";

import React from 'react';

import UserLogin from '../components/UserLogin';

import UserService from '../services/UserService';


export class LoginView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {};
    }

    login(user) {
        UserService.login(user.username, user.password, user.role).then((data) => {
            if (user.role==="user")
                this.props.history.push('/');
            else
                this.props.history.push('/company');
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
    }

    signup(user) {
        UserService.register(user).then((data) => {
            this.props.history.push('/profile');
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
    }

    render() {
        return (
          <UserLogin onLoginSubmit={(user) => this.login(user)} error={this.state.error} onSignupSubmit={(user) => this.signup(user)} error={this.state.error}/>
        );
    }
}
