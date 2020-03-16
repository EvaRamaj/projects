"use strict";

import React from 'react';

import { UserEvaluationDetail } from '../components/UserEvaluationDetail';

import UserEvaluationService from '../services/UserEvaluationService';


export class UserEvaluationDetailView extends React.Component {
    constructor(props) {
        super(props);
    }

    componentWillMount(props){
        this.setState({
            loading: true,
            user_evaluation: []
        });
        let id = this.props.match.params.id;
        console.log(id);
        if(this.props.match.path === '/user_evaluation/:id') {

            UserEvaluationService.getEvaluationDetails(id).then((data) => {
                console.log("hfsdlfh")

                console.log(data.user_evaluation.user_evaluation)
                this.setState({
                    user_evaluation: data.user_evaluation.user_evaluation,
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });
        }


    }



    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <UserEvaluationDetail user_evaluation={this.state.user_evaluation} onDelete={(id) => this.deleteMovie(id)}/>
        );
    }
}
