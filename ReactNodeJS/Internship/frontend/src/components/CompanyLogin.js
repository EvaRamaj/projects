"use strict";

import React from 'react';
import { Card, Button, TextField, Grid, Cell, Subheader, SelectField } from 'react-md';
import { withRouter, Link } from 'react-router-dom';

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import 'react-md/dist/react-md.indigo-pink.min.css'
import {LinkedIn} from "react-linkedin-login-oauth2";
import {Image} from "react-bootstrap";
import linkedInImg from "../assets/img/LinkedIn.png";
import UserService from "../services/UserService";


const style = { maxWidth: 500 };


class CompanyLogin extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            signupemail : '',
            signuppassword : '',
            companyname: '',
            firstname: '',
            lastname: '',
            prefix: '',
            linkedInSignUpError: ''
        };

        this.handleChangeSignupEmail = this.handleChangeSignupEmail.bind(this);
        this.handleChangeSignupPassword = this.handleChangeSignupPassword.bind(this);
        this.handleChangeCompanyName = this.handleChangeCompanyName.bind(this);
        this.handleChangeFirstName = this.handleChangeFirstName.bind(this);
        this.handleChangeLastName = this.handleChangeLastName.bind(this);
        this.handleChangePrefix = this.handleChangePrefix.bind(this);

        this.handleSignUpSubmit = this.handleSignUpSubmit.bind(this);

        this.handleLinkedInFailure = this.handleLinkedInFailure.bind(this);
        this.handleLinkedInSignUpSuccess = this.handleLinkedInSignUpSuccess.bind(this);
    }

    handleChangeSignupEmail(value) {
        this.setState(Object.assign({}, this.state, {signupemail: value}));
    }

    handleChangeSignupPassword(value) {
        this.setState(Object.assign({}, this.state, {signuppassword: value}));
    }

    handleChangeCompanyName(value) {
        this.setState(Object.assign({}, this.state, {companyname: value}));
    }

    handleChangeFirstName(value) {
        this.setState(Object.assign({}, this.state, {firstname: value}));
    }

    handleChangeLastName(value) {
        this.setState(Object.assign({}, this.state, {lastname: value}));
    }

    handleChangePrefix(value) {
        this.setState(Object.assign({}, this.state, {prefix: value}));
    }

    handleSignUpSubmit(event) {
        event.preventDefault();

        let company = {
            role: "company",
            username: this.state.signupemail,
            password: this.state.signuppassword,
            firstName: this.state.firstname,
            lastName: this.state.lastname,
            prefix: this.state.prefix,
            title: this.state.companyname
        };

        this.props.onSignupSubmit(company);
    }

    handleLinkedInFailure(error)
    {
        this.setState(Object.assign({}, this.state, {linkedInSignUpError: error.errorMessage}));
    }
    handleLinkedInSignUpSuccess(response)
    {
        UserService.linkedIn(response.code,'linkedinSignup','company').then((data) => {
            this.props.history.push('/company/search');
        }).catch((e) => {
            this.setState({
                linkedInSignUpError: e
            });
            console.error(e);
        });
    }

    render() {
        const ITEMS_WITH_ELEMENTS = [
            <Subheader key="subheader-1" primaryText="Prefix" className="md-divider-border md-divider-border--bottom" />,
            'Mr.', 'Ms.', 'Mrs.'
        ];
        return (
            <Page isStickyFooter={true}>
                <div className="col-md-4"/>
                <div className="col-md-4">
                    <Card>
                        <h2 class="text-center">Register here to see the Talent Pool</h2>
                        <div className="text-center">
                            <LinkedIn
                                clientId="7731zhx8b7r8sx"
                                onFailure={this.handleLinkedInFailure}
                                onSuccess={this.handleLinkedInSignUpSuccess}
                                redirectUri={CONFIG.App.FrontendServerIP.protocol+"://"+CONFIG.App.FrontendServerIP.host+":"+CONFIG.App.FrontendServerIP.port+"/auth/linkedin/callback"}
                                className="linkedInBtn"
                            >
                                <Image src={linkedInImg} style={{height: 100+'%',width: "auto"}}/>
                            </LinkedIn>
                            <AlertMessage className="md-row md-full-width" >{this.state.linkedInSignUpError ? `${this.state.linkedInSignUpError}` : ''}</AlertMessage>
                        </div>
                        <form className="md-grid" onSubmit={this.handleSignUpSubmit} onReset={() => this.props.history.goBack()}>
                            <SelectField
                                id="prefixSelectField"
                                label="Prefix"
                                placeholder="Select something"
                                menuItems={ITEMS_WITH_ELEMENTS}
                                className="md-cell md-cell--4"
                                sameWidth
                                value={this.state.prefix}
                                onChange={this.handleChangePrefix}
                            />
                            <TextField
                                label="First Name"
                                id="fnField"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.firstname}
                                onChange={this.handleChangeFirstName}
                                errorText="First Name is required"/>
                            <TextField
                                label="Last Name"
                                id="lnField"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.lastName}
                                onChange={this.handleChangeLastName}
                                errorText="Last Name is required"/>
                            <TextField
                                label="Company Name"
                                id="cnField"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.companyname}
                                onChange={this.handleChangeCompanyName}
                                errorText="Company Name is required"/>
                            <TextField
                                label="Work Email"
                                id="emailSUField"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.signupemail}
                                onChange={this.handleChangeSignupEmail}
                                errorText="Email is required"/>
                            <TextField
                                label="Password"
                                id="passwordSUField"
                                type="password"
                                className="md-row"
                                required={true}
                                value={this.state.signuppassword}
                                onChange={this.handleChangeSignupPassword}
                                errorText="Password is required"/>

                            <Button id="submit" type="submit"
                                    disabled={this.state.signupemail == undefined || this.state.signupemail == '' || this.state.signuppassword == undefined || this.state.signuppassword == ''
                                    || this.state.firstname == undefined || this.state.firstname == '' || this.state.lastname == undefined || this.state.lastname == ''
                                    || this.state.companyname == undefined || this.state.companyname == '' ? true : false}
                                    raised primary className="md-cell md-cell--2">SignUp</Button>
                            <Button id="resetSignup" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
                            <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                        </form>
                    </Card>
                </div>
                <div className="col-md-4"/>
            </Page>
        );
    }
};

export default withRouter(CompanyLogin);
