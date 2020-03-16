"use strict";

import React from 'react';
// import { Card, Button, FontIcon, TextField } from 'react-md';
import { withRouter } from 'react-router-dom'

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

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
    Input,
    Label,
    FormText,
    FormGroup,
    Form,
    Row,
    Table,
} from 'reactstrap';

const style = { maxWidth: 500 };

class UserForm extends React.Component {

    constructor(props) {
        super(props);
        console.log("props: ", props);
        if(this.props.user != undefined) {
            this.state = {
                username : props.user.username,
                password : props.user.password,
                email : props.user.email,
                first_name : props.user.first_name,
                last_name : props.user.last_name,
                address : props.user.address,
                phone : props.user.phone,
                photo : props.user.photo,
                id_document : props.user.id_document,
                lessor_role_requested : props.user.lesor_role_requested,
                notification: false
            };
        } else {
            this.state = {
                username : '',
                password : '',
                email : '',
                first_name : '',
                last_name : '',
                address : '',
                phone : '',
                photo : '',
                id_document : '',
                lessor_role_requested : '',
            };
        }

        this.handleChangeUsername = this.handleChangeUsername.bind(this);
        this.handleChangePassword = this.handleChangePassword.bind(this);
        this.handleChangeEmail = this.handleChangeEmail.bind(this);
        this.handleChangeFirstName = this.handleChangeFirstName.bind(this);
        this.handleChangeLastName = this.handleChangeLastName.bind(this);
        this.handleChangeAddress = this.handleChangeAddress.bind(this);
        this.handleChangePhone = this.handleChangePhone.bind(this);
        this.handleChangePhoto = this.handleChangePhoto.bind(this);
        this.handleChangeIdDocument = this.handleChangeIdDocument.bind(this);
        this.handleChangeRole = this.handleChangeRole.bind(this);
        this.handleChangeIsLessor = this.handleChangeIsLessor.bind(this);
        this.handleChangeLessorRoleRequested = this.handleChangeLessorRoleRequested.bind(this);


        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChangeUsername(event) {
        this.setState(Object.assign({}, this.state, {username: event.target.value}));
    }
    handleChangePassword(event) {
        this.setState(Object.assign({}, this.state, {password: event.target.value}));
    }
    handleChangeEmail(event) {
        this.setState(Object.assign({}, this.state, {email: event.target.value}));
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
        this.setState(Object.assign({}, this.state, {phone: event.target.value}));
    }
    handleChangePhoto(event) {
        this.setState(Object.assign({}, this.state, {photo: event.target.value}));
    }
    handleChangeIdDocument(event) {
        this.setState(Object.assign({}, this.state, {id_document: event.target.value}));
    }
    handleChangeRole(event) {
        this.setState(Object.assign({}, this.state, {role: event.target.value}));
    }
    handleChangeIsLessor(event) {
        this.setState(Object.assign({}, this.state, {role: event.target.value}));
    }
    handleChangeLessorRoleRequested(event) {
        this.setState(Object.assign({}, this.state, {lessor_role_requested: event.target.value}));
    }

    handleSubmit(event) {
        event.preventDefault();

        let user = this.props.user;
        if(user == undefined) {
            user = {};
        }
        for (let key in user){
            user[key] = this.state[key]
        }
        console.log('user: ',this.state);

        this.notify();
        let prop =  this.props;
        setTimeout(function() {
            prop.onSubmit(user);
        }, 1000);

    }

    notify() {
        // toast("Default Notification !");

        toast.success("Profile updated", {
            position: toast.POSITION.TOP_CENTER
        });
        this.state.notification = true;
    }

    render() {
        let user = this.state;
        console.log('here: ', user);
        let self = this; // otherwise it doesn't see this obj
        return (
            <Page>
                <Row>
                <Col>
                <Card>
                    <CardHeader>
                         <strong>{user.username}</strong> Here you can edit your Profile
                    </CardHeader>
                    <CardBody>
                        <Form onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()} className="form-horizontal">

                            {Object.keys(user).map(function(key) {
                                    if (key !== 'email' && key !== 'password' && key !== 'lessor_role_requested' && key !== '__v' && key !== '_id' && key !== 'photo' && key !== 'id_document' && key !== 'notification') {
                                        // parse key and process it in order to form handlers name
                                        let s = key.toString();
                                        let out = s.split("_");
                                        let handler = 'handleChange';
                                        let label = '';
                                        for (let w in out){
                                            handler += out[w].substring(0, 1).toUpperCase() + out[w].substring(1);
                                            label +=' ' + out[w].substring(0, 1).toUpperCase() + out[w].substring(1);
                                        }
                                        // cases to take into account in order to iterate and display text boxes
                                        // let label = key;
                                        let id = key.toString().toLocaleUpperCase() + 'Field';
                                        return (

                                            <FormGroup row>
                                                <Col md="3">
                                                    <Label htmlFor="text-input">{label}:</Label>
                                                </Col>
                                                <Col xs="12" md="9">
                                                    <Input type="text" id="text-input" name="text-input"
                                                           placeholder="Text" onChange={self[handler]} required={true} value={user[key]}/>
                                                    {/*<FormText color="muted">This is a help text</FormText>*/}
                                                </Col>
                                            </FormGroup>
                                        );
                                    }
                                    if(key === 'email'){
                                        return(
                                            <FormGroup row>
                                                <Col md="3">
                                                    <Label htmlFor="email-input">Email</Label>
                                                </Col>
                                                <Col xs="12" md="9">
                                                    <Input type="email" id="email-input" name="email-input" onChange={self.handleChangeEmail} required={true} placeholder="Enter Email" autoComplete="email" value={user[key]}/>
                                                    {/*<FormText className="help-block">Please enter your email</FormText>*/}
                                                </Col>
                                            </FormGroup>
                                        );
                                    }
                                    if(key === 'password'){
                                        return(
                                            <FormGroup row>
                                                <Col md="3">
                                                    <Label htmlFor="password-input">Password</Label>
                                                </Col>
                                                <Col xs="12" md="9">
                                                    <Input type="password" id="password-input" onChange={self.handleChangePassword} required={true} name="password-input" placeholder="Password" autoComplete="new-password" value={user[key]}/>
                                                    {/*<FormText className="help-block">Please enter a complex password</FormText>*/}
                                                </Col>
                                            </FormGroup>
                                        );
                                    }
                                }
                                )
                            }
                            <Button type="submit" size="sm" color="success"><i className="fa fa-check"></i> Submit</Button>
                            <Button type="reset" size="sm" color="danger"><i className="fa fa-times"></i> Cancel</Button>


                        </Form>
                    </CardBody>
                </Card>

                </Col>
                </Row>
                {/*<Card style={style} className="md-block-centered">*/}
                    {/*<form className="md-grid" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>*/}
                        {/*{Object.keys(user).map(function(key) {*/}
                            {/*if(key !== 'lessor_role_requested' && key !== '__v' && key !== '_id' && key !== 'photo' && key !== 'id_document' && key !== 'notification'){*/}

                                {/*// parse key and process it in order to form handlers name*/}
                                {/*let s = key.toString();*/}
                                {/*let out = s.split("_");*/}
                                {/*let handler = 'handleChange';*/}
                                {/*for (let w in out){*/}
                                    {/*handler += out[w].substring(0, 1).toUpperCase() + out[w].substring(1);*/}
                                {/*}*/}

                                {/*// cases to take into account in order to iterate and display text boxes*/}
                                {/*let label = key;*/}
                                {/*let id = key.toString().toLocaleUpperCase() + 'Field';*/}
                                {/*let required = false;*/}
                                {/*let type = "text";*/}

                                {/*if(key === 'password'){*/}
                                    {/*type = "password";*/}
                                    {/*required = true;*/}
                                {/*}*/}
                                {/*if(key === 'username' || key === 'email'){*/}
                                    {/*required = true;*/}
                                {/*}*/}
                                {/*if(key === 'is_Lessor'){*/}
                                    {/*type = "bool";*/}
                                {/*}*/}
                                {/*let value;*/}
                                {/*if(user[key]){*/}
                                    {/*value =user[key];*/}
                                {/*}*/}
                                {/*let errorText;*/}
                                {/*if(required === true){*/}
                                    {/*errorText = key.toString() + ' is required!';*/}
                                {/*}*/}


                                {/*return <TextField*/}
                                    {/*label={label}*/}
                                    {/*id={id}*/}
                                    {/*type={type}*/}
                                    {/*className="md-row"*/}
                                    {/*required={required}*/}
                                    {/*value={value}*/}
                                    {/*onChange={self[handler]}*/}
                                    {/*errorText={errorText}/>*/}

                            {/*}*/}

                        {/*})}*/}

                        {/*<Button id="submit" type="submit"*/}
                                {/*disabled={this.state.notification || this.state.username == undefined || this.state.username == '' || this.state.password == undefined || this.state.password == '' || this.state.email == undefined || this.state.email == '' ? true : false}*/}
                                {/*raised primary className="md-cell md-cell--2">Save</Button>*/}
                        {/*<ToastContainer autoClose={1000}/>*/}
                        {/*<Button disabled={this.state.notification} id="reset" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>*/}
                        {/*<AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>*/}
                    {/*</form>*/}
                {/*</Card>*/}
            </Page>
        );
    }
}

export default withRouter(UserForm);