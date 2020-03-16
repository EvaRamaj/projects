"use strict";

import React from 'react';

import BecomeLessorForm from './../components/BecomeLessorForm';

import UserService from '../services/UserService';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
/*
 * assuming the API returns something like this:
 *   const json = [
 *      { value: 'one', label: 'One' },
 *      { value: 'two', label: 'Two' }
 *   ]
 */

export class BecomeLessorFormView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            user: undefined,
            loading: true,
            error: undefined
        };
    }

    componentWillMount(){
        UserService.getMyProfile().then((data) => {
            this.setState({
                user: data,
                loading: false,
                error: undefined
            });
        }).catch((e) => {
            console.error(e);
        });
    }


    requestUpgrade(user) {
            UserService.requestUpgrade(user).then((data) => {
                this.props.history.goBack();
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating movie'}));
            });
    }
    notify() {
        // toast("Default Notification !");

        toast.warn("Patience you must have my Young Padawan", {
            position: toast.POSITION.TOP_CENTER
        });
        // this.state.notification = true;
    }

    render() {
        if(this.state.loading){
            return(
                <h2>Loading...</h2>
            );
        }
        if(!this.state.user.lessor_role_requested){
            return (
                <BecomeLessorForm user = {this.state.user} onSubmit={(user) => this.requestUpgrade(user)} error={this.state.error} />
            );
        }
        else{
            return (
                <h6>Patience you must have my Young Padawan</h6>
                // <ToastContainer autoClose={1000}/>
            );
        }
    }
}
