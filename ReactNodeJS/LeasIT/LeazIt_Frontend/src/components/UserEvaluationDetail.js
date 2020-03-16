"use strict";

import React from 'react';
import { Link } from 'react-router-dom'
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon } from 'react-md';

import Page from './Page';

import AuthService from '../services/AuthService';

export class UserEvaluationDetail extends React.Component {

    constructor(props) {
        super(props);
    }

    render() {

        let user_evaluation = this.props.user_evaluation


        return (
            <Page>
                <b><font size="5" face="verdana" color="black">Evaluation Details</font></b>
                <Card>

                    <b>
                        <CardText>
                            {Object.keys(user_evaluation).map(function(key) {
                                console.log("key",key);
                                /* if (key === "booker_id"){
                                      return <div><font size="5" face="verdana" color="black">{booking[key].username} {booking[key]._id}</font></div>
                                 }*/

                                    console.log("fhkslhf");
                                    console.log(user_evaluation[key])
                                    return( <div>
                                        <p>Evaluator: {user_evaluation[key].evaluator_id.username}</p>
                                        <p>Rating: {user_evaluation[key].rating}</p>
                                        <p>Comment: {user_evaluation[key].comment}</p>

                                    </div>);





                                return //<div>{key}: {booking[key]}</div>
                            })}
                        </CardText></b>
                </Card>
            </Page>
        );
    }
}