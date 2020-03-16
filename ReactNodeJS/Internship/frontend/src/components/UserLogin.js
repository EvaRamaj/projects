"use strict";

import React from 'react';
import {Card, Button, TextField, Subheader, SelectField, SelectionControlGroup, DialogContainer} from 'react-md';
import { withRouter, Link } from 'react-router-dom';

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import 'react-md/dist/react-md.indigo-pink.min.css'
import linkedInImg from "./../assets/img/LinkedIn.png";
import {Image} from "react-bootstrap";
import { LinkedIn } from 'react-linkedin-login-oauth2';
import UserService from "../services/UserService";

const style = { maxWidth: 500 };

class UserLogin extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loginemail : '',
            loginpassword : '',
            loginrole: 'user',
            signupemail : '',
            signuppassword : '',
            firstname: '',
            lastname: '',
            prefix: '',
            linkedInSignUpError: '',
            emailValid: ''
        };

        this.handleChangeLoginEmail = this.handleChangeLoginEmail.bind(this);
        this.handleChangeLoginPassword = this.handleChangeLoginPassword.bind(this);
        this.handleChangeRole = this.handleChangeRole.bind(this);

        this.handleChangeSignupEmail = this.handleChangeSignupEmail.bind(this);
        this.handleChangeSignupPassword = this.handleChangeSignupPassword.bind(this);
        this.handleChangeFirstName = this.handleChangeFirstName.bind(this);
        this.handleChangeLastName = this.handleChangeLastName.bind(this);
        this.handleChangePrefix = this.handleChangePrefix.bind(this);

        this.handleLoginSubmit = this.handleLoginSubmit.bind(this);
        this.handleSignUpSubmit = this.handleSignUpSubmit.bind(this);

        this.handleLinkedInFailure = this.handleLinkedInFailure.bind(this);
        this.handleLinkedInSignUpSuccess = this.handleLinkedInSignUpSuccess.bind(this);
        this.handleLinkedInLoginSuccess = this.handleLinkedInLoginSuccess.bind(this);
    }
    validateField(value) {
        let fieldValidationErrors = this.state.formErrors;
        let emailValid = this.state.emailValid;


        emailValid = value.match(/^([\w.%+-]+)@([\w-]+\.)+([\w]{2,})$/i);
        fieldValidationErrors.signupemail = emailValid ? '' : ' is invalid';

        this.setState({formErrors: fieldValidationErrors,
            emailValid: emailValid,
        }, this.validateForm);
    }

    validateForm() {
        this.setState({formValid: this.state.emailValid});
    }

    handleChangeLoginEmail(value) {
        this.setState(Object.assign({}, this.state, {loginemail: value}));
    }

    handleChangeLoginPassword(value) {
        this.setState(Object.assign({}, this.state, {loginpassword: value}));
    }

    handleChangeRole(value) {
        this.setState(Object.assign({}, this.state, {loginrole: value}));
    }

    handleChangeSignupEmail(value) {
        console.log(value);
        this.setState({signupemail: value},(value) => { this.validateField(value) });
       //this.setState(Object.assign({}, this.state, {signupemail: value}));
    }

    handleChangeSignupPassword(value) {
        this.setState(Object.assign({}, this.state, {signuppassword: value}));
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

    handleLoginSubmit(event) {
        event.preventDefault();

        let user = {
            username: this.state.loginemail,
            password: this.state.loginpassword,
            role: this.state.loginrole
        };

        this.props.onLoginSubmit(user);
    }

    handleSignUpSubmit(event) {
        event.preventDefault();

        let user = {
            role: 'user',
            username: this.state.signupemail,
            password: this.state.signuppassword,
            profileData: {
                firstName: this.state.firstname,
                lastName: this.state.lastname,
                prefix: this.state.prefix
            }
        };

        this.props.onSignupSubmit(user);
    }

    handleLinkedInFailure(error)
    {
        this.setState(Object.assign({}, this.state, {linkedInSignUpError: error.errorMessage}));
    }
    handleLinkedInSignUpSuccess(response)
    {
        UserService.linkedIn(response.code,'linkedinSignup','user').then((data) => {
            this.props.history.push('/profile');
        }).catch((e) => {
            this.setState({
                linkedInSignUpError: e
            });
            console.error(e);
        });
    }
    handleLinkedInLoginSuccess(response)
    {
        UserService.linkedIn(response.code,'linkedinLogin',this.state.loginrole).then((data) => {
            this.props.history.push('/');
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
                <Page id="login-page" isStickyFooter={true}>
                    {/*<DialogContainer
                        id="success-modal"
                        focusOnMount={false}
                        visible={this.props.emailConfirm}
                        title="Confirmation Successful"
                        aria-describedby="success-modal-div"
                        height={300}
                        width={500}
                    >
                        <div id="success-modal-div" style={{padding: 50+'px'}}>
                            <h1 className="text-center md-color--secondary-text">Thank you for confirming your email.</h1>
                        </div>
                    </DialogContainer>*/}
                  <div className="col-md-2"/>
                  <div className="col-md-4">
                    <Card style={{paddingTop:"1.5em"}}>
                    <h2 className="text-center">Sign In to WAYER</h2>
                        <div className="text-center">
                            <LinkedIn
                                clientId="7731zhx8b7r8sx"
                                onFailure={this.handleLinkedInFailure}
                                onSuccess={this.handleLinkedInLoginSuccess}
                                redirectUri={CONFIG.App.FrontendServerIP.protocol+"://"+CONFIG.App.FrontendServerIP.host+":"+CONFIG.App.FrontendServerIP.port+"/auth/linkedin/callback"}
                                className="linkedInBtn"
                            >
                                <Image src={linkedInImg} style={{height: 100+'%',width: "auto"}}/>
                            </LinkedIn>
                            <AlertMessage className="md-row md-full-width" >{this.state.linkedInSignUpError ? `${this.state.linkedInSignUpError}` : ''}</AlertMessage>
                        </div>
                      <form className="md-grid" onSubmit={this.handleLoginSubmit} onReset={() => this.props.history.goBack()}>
                          <TextField
                              label="Email"
                              id="emailLIField"
                              type="text"
                              className="md-row"
                              required={true}
                              value={this.state.loginemail}
                              onChange={this.handleChangeLoginEmail}
                              errorText="Email is required"/>
                          <TextField
                              label="Password"
                              id="passwordLIField"
                              type="password"
                              className="md-row"
                              required={true}
                              value={this.state.loginpassword}
                              onChange={this.handleChangeLoginPassword}
                              errorText="Password is required"/>
                          <div className="col-md-12" style={{paddingBottom: 20+'px'}}>
                              <SelectionControlGroup
                                  id="selection-control-group-radios"
                                  name="radio-role"
                                  className="md-row"
                                  type="radio"
                                  defaultValue="user"
                                  inline
                                  onChange={this.handleChangeRole}
                                  controls={[{
                                      label: 'Retired Professional',
                                      value: 'user'
                                  }, {
                                      label: 'Company',
                                      value: 'company'
                                  }]}
                              />
                          </div>
                          <Button id="submit" type="submit"
                                  disabled={this.state.loginemail == undefined || this.state.loginemail == '' || this.state.loginpassword == undefined || this.state.loginpassword == '' ? true : false}
                                  raised primary className="md-cell md-cell--2">Login</Button>
                          <Button id="resetLogin" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
                          <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                      </form>
                      </Card>
                  </div>
                  <div className="col-md-4">
                    <Card style={{paddingTop:"1.5em"}}>
                    <h2 class="text-center">Sign Up to WAYER</h2>
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
                        {/*<Button flat className="linkedInBtn" onClick={this.handleLinkedInSignUp}><Image src={linkedInImg} style={{height: 100+'%',width: "auto"}}/></Button>*/}

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
                              label="Email"
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
                                    || this.state.firstname == undefined || this.state.firstname == '' || this.state.lastname == undefined || this.state.lastname == '' ? true : false}
                                  raised primary className="md-cell md-cell--2">SignUp</Button>
                                <Button id="resetSignup" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
                          <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                      </form>
                      </Card>
                  </div>
                  <div className="col-md-2"/>
                </Page>
        );
    }
};

export default withRouter(UserLogin);
