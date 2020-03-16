"use strict";

import React from 'react';
import queryString from 'query-string'

import UserService from '../services/UserService';
import Link from "react-router-dom/es/Link";

export class ConfirmEmailView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            emailConfirm: false
        };
    }

    componentDidMount()
    {
        if (this.props.location.search && !this.state.emailConfirm)
        {
            const values = queryString.parse(this.props.location.search);
            UserService.confirmEmail(values.token).then((data) => {
                this.setState({
                    emailConfirm: true
                }, () => {
                    setTimeout(()=>this.props.history.replace('/'),5000)
                });
            }).catch((e) => {
                console.error(e);
            });
        }
    }

    render() {
        return (
            <div>
                <div className="col-md-12" style={{visibility: this.state.emailConfirm ? 'visible' : 'hidden'}}>
                    <h2>Thank you for confirming your email.</h2>
                    <h4>You'll be redirected to the Wayer homepage in 5 seconds. If you are not redirected automatically, Please click <Link to='/'>here</Link>.</h4>
                </div>
                <div className="col-md-12" style={{visibility: !this.state.emailConfirm ? 'visible' : 'hidden'}}><h2>Confirming your email...</h2></div>
            </div>
        );
    }
}