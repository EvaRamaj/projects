"use strict";

import React from 'react';
import { Card, Button, FontIcon, TextField } from 'react-md';
import { withRouter } from 'react-router-dom'

import { AlertMessage } from './AlertMessage';
import Page from './Page';


const style = { maxWidth: 500 };


class UserEvaluationForm extends React.Component {

    constructor(props) {
        super(props);
        console.log("props: ", props);
        if(this.props.user_evaluation != undefined) {
            this.state = {
                comment : props.user_evaluation.comment,
                rate : props.user_evaluation.rate,
                user: props.user_evaluation.evaluatee_id,

            };
        } else {
            this.state = {
                comment : '',
                rate : '',
                evaluatee_id: '',

            };
        }

        this.handleAddComment = this.handleAddComment.bind(this);
        this.handleAddRating = this.handleAddRating.bind(this);


        this.handleAddUser=this.handleAddUser.bind(this);

        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleAddComment(value) {
        this.setState(Object.assign({}, this.state, {comment: value}));
    }

    handleAddRating(value) {
        this.setState(Object.assign({}, this.state, {rating: value}));
    }


    handleAddUser(value){

        this.setState(Object.assign({}, this.state, {evaluatee_id: value}));
    }

    handleSubmit(event) {
        event.preventDefault();
        let user_evaluation= {};

        if(user_evaluation == undefined) {


        }
        for (let key in user_evaluation){
            user_evaluation[key] = this.state[key]
        }
        console.log('user_evaluation: ',this.state);
        user_evaluation.comment = this.state.comment;
        user_evaluation.rating = this.state.rating;

        user_evaluation.evaluatee_id = this.state.evaluatee_id;

        this.props.onSubmit(user_evaluation);
    }


    render() {
        return (
            <Page>
                <Card style={style} className="md-block-centered">
                    <form className="md-grid" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
                        <TextField
                            label="comment"
                            id="TitleField"
                            type="text"
                            className="btn btn-primary"
                            required={true}
                            value={this.state.comment}
                            onChange={this.handleAddComment}

                            errorText="Comment is required"/>

                        <TextField
                            label="rate"
                            id="TitleField"
                            type="number"

                            className="btn btn-primary"
                            required={true}
                            value={this.state.rating}
                            onChange={this.handleAddRating}
                            errorText="Rate from 1 to 5 is only accepted"/>
                        <TextField
                            label="user"
                            id="TitleField"
                            type="text"
                            className="btn btn-primary"
                            required={true}
                            value={this.state.evaluatee_id}
                            onChange={this.handleAddUser}
                            errorText="User is required"/>

                        <Button id="submit" type="submit"
                                 className="md-cell md-cell--2">Save</Button>
                        <Button id="reset" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
                        <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                    </form>
                </Card>
            </Page>
        );
    }


}

export default withRouter(UserEvaluationForm);