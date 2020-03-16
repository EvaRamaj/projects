"use strict";

import React from 'react';

import CompanyLogin from '../components/CompanyLogin';

import UserService from '../services/UserService';


export class CompanySignUpView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {};
    }

    signup(company) {
        UserService.register(company).then((data) => {
            //this.props.history.replace({pathname:'/company/search',state:{searchtext: (this.props.history.location.state ? this.props.history.location.state.searchtext : '')}});
            this.props.history.replace('/company/project/create');
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
    }

    render() {
        return (
            <CompanyLogin onSignupSubmit={(company) => this.signup(company)} error={this.state.error}/>
        );
    }
}
