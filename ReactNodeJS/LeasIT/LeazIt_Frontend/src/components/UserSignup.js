

"use strict";

import React from 'react';
// import { Card, Button, TextField } from 'react-md';
import {
    Badge,
    Button,
    ButtonDropdown,
    ButtonGroup,
    ButtonToolbar,
    Card,
    CardBody,
    CardFooter,
    CardHeader,
    CardTitle,
    Col,
    Dropdown,
    DropdownItem,
    FormText,
    FormGroup,
    Form,
    Row,
    Table,
} from 'reactstrap';
import { withRouter } from 'react-router-dom';

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import axios from "axios";

import PhoneInput from 'react-phone-number-input';
import IntlTelInput from 'react-intl-tel-input';

import { parseNumber, formatNumber, isValidNumber } from 'libphonenumber-js';

import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

const style = { maxWidth: 500 };


class UserSignup extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            username : '',
            password : '',
            confirmPassword: '',
            email : '',
            first_name : '',
            last_name : '',
            address : '',
            phone : '',
            id_document : '',
            lessor_role_requested : false,
            role: 'Lessee',
            is_Lessor: false,
            selectedFile0: undefined,
            formErrors: {email: ''},
            emailValid: false,
            formValid: false,
            notification: false
        };
        this.photoNames = [];
        this.handleChangeUsername = this.handleChangeUsername.bind(this);
        this.handleChangePassword = this.handleChangePassword.bind(this);
        this.onChangePassword = this.onChangePassword.bind(this);
        this.onChangeConfirmPassword = this.onChangeConfirmPassword.bind(this);
        this.handleChangeEmail = this.handleChangeEmail.bind(this);
        this.handleChangeFirstName = this.handleChangeFirstName.bind(this);
        this.handleChangeLastName = this.handleChangeLastName.bind(this);
        this.handleChangeAddress = this.handleChangeAddress.bind(this);
        this.handleChangePhone = this.handleChangePhone.bind(this);
        this.handleChangePhoto0 = this.handleChangePhoto0.bind(this);
        this.handleChangeLessorRoleRequested = this.handleChangeLessorRoleRequested.bind(this);


        this.handleSubmit = this.handleSubmit.bind(this);
        console.log("in2")
    }

    onChangePassword (e){
        let state = this.state;
        console.log(state[e.target.name]);
        state[e.target.name] = e.target.value;
        this.setState(Object.assign({}, this.state, {password: e.target.value}))};
    onChangeConfirmPassword (e){
        let state = this.state;
        console.log(state[e.target.name]);
        state[e.target.name] = e.target.value;
        this.setState(Object.assign({}, this.state, {confirmPassword: event.target.value}))};
    handleChangePhoto0 (event){
        this.setState(Object.assign({}, this.state, {selectedFile0: event.target.files[0]}));
        console.log(this.state);
    };

    handleChangeUsername(event) {
        console.log('username');
        this.setState(Object.assign({}, this.state, {username: event.target.value}));
    }
    handleChangePassword(event) {
        console.log('pass');
        this.setState(Object.assign({}, this.state, {password: event.target.value}));
    }

    handleChangeEmail(value) {
        console.log('email');
        this.setState({email: value},() => { this.validateField(value) });
        // this.setState(Object.assign({}, this.state, {email: value}));
    }

    validateField(value) {
        let fieldValidationErrors = this.state.formErrors;
        let emailValid = this.state.emailValid;


         emailValid = value.match(/^([\w.%+-]+)@([\w-]+\.)+([\w]{2,})$/i);
         fieldValidationErrors.email = emailValid ? '' : ' is invalid';

        this.setState({formErrors: fieldValidationErrors,
            emailValid: emailValid,
        }, this.validateForm);
    }

    validateForm() {
        this.setState({formValid: this.state.emailValid});
    }


    handleChangeFirstName(value) {
        this.setState(Object.assign({}, this.state, {first_name: value}));
    }

    handleChangeEmail(event) {
        console.log(event);
        //this.setState({email: event.target.value},(value) => { this.validateField(value) });
        this.setState(Object.assign({}, this.state, {email: event.target.value}));
    }

    validateField(value) {
        let fieldValidationErrors = this.state.formErrors;
        let emailValid = this.state.emailValid;


        emailValid = value.match(/^([\w.%+-]+)@([\w-]+\.)+([\w]{2,})$/i);
        fieldValidationErrors.email = emailValid ? '' : ' is invalid';

        this.setState({formErrors: fieldValidationErrors,
            emailValid: emailValid,
        }, this.validateForm);
    }

    validateForm() {
        this.setState({formValid: this.state.emailValid});
    }


    handleChangeFirstName(event) {
        this.setState(Object.assign({}, this.state, {first_name: event.target.value}));
    }

    handleChangeLastName(event) {
        this.setState(Object.assign({}, this.state, {last_name: event.target.value}));
    }
    handleChangeAddress(event) {
        this.setState(Object.assign({}, this.state, {address: event.target.value}));
    }
    handleChangePhone(event) {
        const re = /^[0-9\b]+$/;
        if (event.target.value === '' || re.test(event.target.value)) {
            this.setState(Object.assign({}, this.state, {phone: event.target.value}));
        }

    }

    handleChangeLessorRoleRequested(value) {
        this.setState(Object.assign({}, this.state, {lessor_role_requested: value}));
    }

    handleSubmit(event) {
        console.log("haris",event);
        event.preventDefault();
        if (this.state.selectedFile0 !== undefined) {
            const formData = new FormData();
            formData.append('photo', this.state.selectedFile0);
            axios.post('http://localhost:3000/photos', formData).then(
                response => {
                    this.photoNames.push(response.data.file.filename);
                    let user = {
                        username: this.state.username,
                        password: this.state.password,
                        email: this.state.email,
                        first_name: this.state.first_name,
                        last_name: this.state.last_name,
                        address: this.state.address,
                        phone: this.state.phone,
                        photo: this.photoNames,
                        id_document: '',
                        lessor_role_requested: this.state.lessor_role_requested,
                        role: this.state.role,
                        is_Lessor: this.state.is_Lessor,
                    };


                    this.notify();
                    this.setState(Object.assign({}, this.state, {photoNames: this.photoNames}));
                    // console.log("before submission: ", user);
                    let prop =  this.props;
                    setTimeout(function() {
                        prop.onSubmit(user);
                    }, 1500);

                }
            )
        }
        else {
            let user = {
                username: this.state.username,
                password: this.state.password,
                email: this.state.email,
                first_name: this.state.first_name,
                last_name: this.state.last_name,
                address: this.state.address,
                phone: this.state.phone,
                photo: this.photoNames,
                id_document: '',
                lessor_role_requested: this.state.lessor_role_requested,
                role: this.state.role,
                is_Lessor: this.state.is_Lessor
            };
            console.log("haris",this.state);
            this.notify();
            this.setState(Object.assign({}, this.state, {photoNames: this.photoNames}));
            // console.log("before submission: ", user);
            let prop =  this.props;
            setTimeout(function() {
                prop.onSubmit(user);
            }, 1500);
            this.props.onSubmit(user);
        }
    }
    notify() {
        // toast("Default Notification !");

        toast.success("Successfully registered", {
            position: toast.POSITION.TOP_CENTER
        });
        this.notification = true;
    }
    render() {
        let user = this.state;
        let self = this; // otherwise it doesn't see this obj
        return (
            <Page>
                <Row>

                    <Col xs="12" md="12">
                <Card>
                    <CardBody >
                        <Row><Col><br/></Col></Row>
                        <Row>
                            <Col xs="12" md="1"/>
                            <Col xs="12" md="11" className="register ">
                    {/*<div className="register ">*/}
                            <form onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
                                <div className="row">
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField">First Name* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="First Name"
                                            id="first_name"
                                            type="text"
                                            className="md-row"
                                            required={true}
                                            value={this.state.first_name}
                                            onChange={this.handleChangeFirstName}
                                            errorText="Title is required"/>
                                    </div>
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField">Last Name* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="Last Name"
                                            id="last_name"
                                            type="text"
                                            className="md-row"
                                            required={true}
                                            value={this.state.last_name}
                                            onChange={this.handleChangeLastName}
                                            errorText="Title is required"/>
                                    </div>
                                </div>
                                <div className="row">
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField"> E-mail* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="E-mail"
                                            id="email"
                                            type="text"
                                            className="md-row"
                                            required={true}
                                            value={this.state.email}
                                            onChange={this.handleChangeEmail}
                                            errorText="Title is required"/>
                                    </div>
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField">Username* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="username"
                                            id="username"
                                            type="text"
                                            className="md-row"
                                            required={true}
                                            value={this.state.username}
                                            onChange={this.handleChangeUsername}
                                            errorText="Title is required"/>
                                    </div>
                                </div>
                                <div className="row">
                                    {/*<div className="form-group col-md-6">*/}
                                        {/*<label htmlFor="TitleField">Username* :</label>*/}
                                        {/*<input*/}
                                            {/*className="form-control"*/}
                                            {/*placeholder="username"*/}
                                            {/*id="username"*/}
                                            {/*type="text"*/}
                                            {/*className="md-row"*/}
                                            {/*required={true}*/}
                                            {/*value={this.state.username}*/}
                                            {/*onChange={this.handleChangeName}*/}
                                            {/*errorText="Title is required"/>*/}
                                    {/*</div>*/}
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField">Password* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="password"
                                            id="password"
                                            type="password"
                                            className="md-row"
                                            required={true}
                                            value={this.state.password}
                                            onChange={this.onChangePassword}
                                            errorText="Title is required"/>
                                    </div>
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField">Confirm Password* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="confirm password"
                                            id="confirmPassword"
                                            type="Password"
                                            className="md-row"
                                            required={true}
                                            value={this.state.confirmPassword}
                                            onChange={this.onChangeConfirmPassword}
                                            errorText="Title is required"/>
                                    </div>
                                </div>
                                <div className="row">
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField">Address* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="Address"
                                            id="address"
                                            type="text"
                                            className="md-row"
                                            required={true}
                                            value={this.state.address}
                                            onChange={this.handleChangeAddress}
                                            errorText="Title is required"/>
                                    </div>
                                    <div className="form-group col-md-6">
                                        <label htmlFor="TitleField">Phone* :</label>
                                        <input
                                            className="form-control"
                                            placeholder="Phone"
                                            id="phone"
                                            type="text"
                                            className="md-row"
                                            required={true}
                                            value={this.state.phone}
                                            onChange={this.handleChangePhone}
                                            errorText="Title is required"/>
                                    </div>
                                </div>
                                <div className="row">
                                    <div className="form-group col-md-6">
                                        <label htmlFor="file">Upload a photo*:</label>
                                        <input type="file" required={false} onChange={this.handleChangePhoto0}/>
                                    </div>
                                    <div className="form-group col-xs-6">
                                        <div className="row">
                                            <div className="form-group col-xs-6">
                                                <Button id="submit" type="submit"
                                                        disabled={!this.state.username == undefined || this.state.username == '' || this.state.password == undefined || this.state.password == '' || this.state.email == '' ? true : false}
                                                        raised primary className="md-cell md-cell--1" style={{marginTop: "1.9em", marginLeft:"30px", backgroundColor:"green"}}>Register</Button>
                                                <ToastContainer autoClose={1500}/>
                                            </div>
                                            <div className="form-group col-xs-6">
                                                <Button id="reset" type="reset" raised secondary className="md-cell md-cell--1" style={{marginTop: "1.9em", backgroundColor:"red"}}>Dismiss</Button>
                                                <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </form>
                    {/*</div>*/}
                            </Col>
                        </Row>
                </CardBody>
                </Card>
                    </Col>
                </Row>
            </Page>
        );
    }
}

export default withRouter(UserSignup);