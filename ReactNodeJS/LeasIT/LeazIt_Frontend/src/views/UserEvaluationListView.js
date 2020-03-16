"use strict";

import React from 'react';

import { UserEvaluationList } from '../components/UserEvaluationList';

import UserEvaluationService from '../services/UserEvaluationService';


export class UserEvaluationListView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            data: []
        };
    }

    componentWillMount(){
        this.setState({
            loading: true
        });

        UserEvaluationService.getUserEvaluations().then((data) => {
            console.log(data)
            this.setState({
                data: [...data.evaluations],
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }


    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <UserEvaluationList data={this.state.data} onDelete={(id) => this.deleteUserEvaluation(id)}/>
        );
    }
}
