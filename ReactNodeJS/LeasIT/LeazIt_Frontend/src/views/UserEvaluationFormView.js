"use strict";

import React from 'react';

import UserEvaluationForm from '../components/UserEvaluationForm';

import UserEvaluationService from '../services/UserEvaluationService';
import UserService from '../services/UserService';


export class UserEvaluationFormView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(){
        if(this.props.location.state != undefined && this.props.location.state.user != undefined) {
            this.setState({
                loading: false,
                user: this.props.location.state.user_evaluation,
                error: undefined
            });
        }
        else {
            this.setState({
                loading: false,
                error: undefined
            });


        }
    }
    createUserEvaluation(user_evaluation) {

        UserService.getUserByUsername(user_evaluation.evaluatee_id).then((data) => {
                user_evaluation.evaluatee_id = data;
            console.log(user_evaluation.evaluatee_id);
            UserEvaluationService.createUserEvaluation(user_evaluation).then((data) => {
                console.log(data);
                this.setState({
                    user: data,
                    loading: false,
                    error: undefined
                });
            }).catch((e) => {
                console.error(e);
            });
        }).catch((e) => {
            console.error(e);
            this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
        })
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (<UserEvaluationForm onSubmit={(user_evaluation) => this.createUserEvaluation(user_evaluation)} error={this.state.error} />);
    }

}