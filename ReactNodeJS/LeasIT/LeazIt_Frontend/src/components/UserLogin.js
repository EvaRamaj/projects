"use strict";

import React from 'react';
import { withRouter, Link } from 'react-router-dom';

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import logo from '../images/logo.png';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

const style = { maxWidth: 520};


class UserLogin extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            username : '',
            password : '',
            notification: false
        };

        this.handleChangeUsername = this.handleChangeUsername.bind(this);
        this.handleChangePassword = this.handleChangePassword.bind(this);

        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChangeUsername(event) {
        console.log("change",event.target.value);
        this.setState(Object.assign({}, this.state, {username: event.target.value}));
    }

    handleChangePassword(event) {
        this.setState(Object.assign({}, this.state, {password: event.target.value}));
    }

    handleSubmit(event) {
        event.preventDefault();

        let user = {
            username: this.state.username,
            password: this.state.password
        };
        this.notify();
        let prop =  this.props;
        setTimeout(function() {
            prop.onSubmit(user);
        }, 1000);

    }

    notify() {
        // toast("Default Notification !");

        toast.success("Successfully logged in", {
            position: toast.POSITION.TOP_CENTER
        });
        this.state.notification = true;
    }

    render() {
        return (
            <Page>
                <div style={style} className="md-block-centered" id="id01" >
                    <form className="modal-content animate" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
                        <div className="imgcontainer">
                            {/*<span onClick={document.getElementById('id01').style.display='none'} className="close"*/}
                                  {/*title="Close Modal">&times;</span>*/}
                            <img src={logo} alt="Avatar" className="avatar"/>
                        </div>
                            <div className="container" >
                                <label htmlFor="uname"><b>Username</b></label>
                                <input
                                label="Login"
                                id="LoginField"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.username}
                                onChange={this.handleChangeUsername}
                                errorText="Login is required"/>
                            </div>
                            <div className="container" >
                                <label htmlFor="psw"><b>Password</b></label>
                                <input
                                    label="Password"
                                    id="PasswordField"
                                    type="password"
                                    className="md-row"
                                    required={true}
                                    value={this.state.password}
                                    onChange={this.handleChangePassword}
                                    errorText="Password is required"/>
                            </div>
                            <div className="container" >
                            <button id="submit" type="submit"
                                    disabled={this.state.username == undefined || this.state.username == '' || this.state.password == undefined || this.state.password == '' ? true : false}
                                    raised primary className="md-cell md-cell--2">Login</button>
                            </div>
                        <ToastContainer autoClose={1000}/>
                        <div className="container" >
                            <button id="reset" type="reset" raised secondary className="cancelbtn">Cancel</button>
                            </div>
                        <Link to={'/register'} className="md-cell">Not registered yet?</Link>
                        <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                    </form>
                </div>

            </Page>
        );
    }
};

export default withRouter(UserLogin);