"use strict";

import React from 'react';

import Navigation from "../components/Navigation";
import AuthService from "../services/AuthService";


export class NavigationView extends React.Component {

    constructor(props) {
        super(props);

    }

    componentWillMount(props){
        this.currentUser = AuthService.getCurrentUser();
        // console.log("current",this.currentUser)
        this.setState({
            loading: false
        });
    }

    logout() {
        AuthService.logout();
        this.state = {
            user: AuthService.isAuthenticated() ? AuthService.getCurrentUser() : undefined
        };
        if(this.props.location.pathname != '/') {
            this.props.history.push('/');
        }
        else {
            window.location.reload();
        }
    }
    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <Navigation user={this.currentUser} onclick={() => this.logout()}/>
        );
    }
}
