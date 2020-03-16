"use strict";

import React from 'react';

import UserLogout from '../components/UserLogout';

import AuthService from '../services/AuthService';
import createBrowserHistory from 'history/createBrowserHistory';

const history = createBrowserHistory({forceRefresh:true});

export class LogoutView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {};
    }

    logout() {
        AuthService.logout();
        this.state = {
            user: AuthService.isAuthenticated() ? AuthService.getCurrentUser() : undefined
        };
        console.log("token",window.localStorage['jwtToken']);
        console.log("hist",this.props.history);
        if(this.props.location.pathname !== '/') {

            history.push('/');
        }
        else {
            window.location.reload();
        }
    }

    render() {
        return (
            <form className="md-grid" onSubmit={this.logout()} onReset={() => this.props.history.goBack()}>
                <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
            </form>
        );
    }
}