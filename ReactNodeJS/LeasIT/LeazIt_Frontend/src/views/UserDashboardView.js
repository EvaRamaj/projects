"use strict";

import React from 'react';

import {UserDashboard} from "../components/UserDashboard";
import AuthService from "../services/AuthService";
import UserService from "../services/UserService";
import UserEvaluationService from "../services/UserEvaluationService";
import axios from "axios/index";


export class UserDashboardView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });

        UserService.getMyProfile().then((data) => {
            console.log('data:', data);
            UserEvaluationService.getUserEvaluation(data._id).then((user_evals) => {
                let sum = 0;
                for (let i = 0; i < user_evals.evaluations.length; i++) {
                    sum += user_evals.evaluations[i].rating; //don't forget to add the base
                }
                let avg = 0;
                if (user_evals.evaluations.length > 0) {
                    avg = sum / user_evals.evaluations.length;
                }
                this.setState({
                    avg: avg,
                    user: data,
                    loading: false
                });
                console.log('state: ', this.state);
            }).catch((e) => {
                console.error(e);
            });
        }).catch((e) => {
            console.error(e);
        });



    }
    deleteMyProfile() {
        UserService.deleteMyProfile().then((message) => {
            this.props.history.push('/');
        }).catch((e) => {
            console.log(e);
        });
        AuthService.logout();
    }

    requestUpgrade(user) {
        UserService.requestUpgrade(user).then((data) => {
            window.location.reload();
        }).catch((e) => {
            console.error(e);
            this.setState(Object.assign({}, this.state, {error: 'Error while creating movie'}));
        });
    }


    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <UserDashboard user={this.state.user} evaluation={this.state.avg} onSubmit={(user) => this.requestUpgrade(user)} onDelete={() => this.deleteMyProfile()}/>
    );
    }
}
