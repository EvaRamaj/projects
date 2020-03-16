"use strict";

import React from 'react';
import { Card, Button, TextField } from 'react-md';
import { withRouter, Link } from 'react-router-dom';

import { AlertMessage } from './AlertMessage';
import Page from './Page';


const style = { maxWidth: 500 };


class UserLogout extends React.Component {

    constructor(props) {
        super(props);
        this.state={
            hidden : false
        }
        this.handleSubmit = this.handleSubmit.bind(this);
    }


    handleSubmit(event) {
        this.props.onSubmit();
    }

    render() {
        return (
            <form className="md-grid" onSubmit={this.handleSubmit()} onReset={() => this.props.history.goBack()}>
                <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
            </form>
        );
    }
};

export default withRouter(UserLogout);