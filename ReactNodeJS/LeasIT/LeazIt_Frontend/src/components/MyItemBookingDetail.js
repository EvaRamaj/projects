"use strict";

import React from 'react';
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon } from 'react-md';
import { Jumbotron } from 'reactstrap';
import { Form, FormGroup, Label, Input } from 'reactstrap';
import { SimpleLink } from './SimpleLink';
import StarRatingComponent from "react-star-rating-component"

import Page from './Page';

import UserEvaluationService from '../services/UserEvaluationService';
import {withRouter} from "react-router-dom";

class MyItemBookingDetail extends React.Component {

    constructor(props) {
        super(props);
        this.state ={
            avg: 0,
            rating: '',
            comment: ''
        }
        this.handleChangeRating = this.handleChangeRating.bind(this);
        this.handleChangeComment = this.handleChangeComment.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);

    }
    componentWillMount() {
        let booking = this.props.booking;
        UserEvaluationService.getUserEvaluation(booking.booker_id._id).then((user_evals) => {
            var sum = 0;
            for (var i = 0; i < user_evals.length; i++) {
                sum += parseInt(user_evals[i].rating, 10); //don't forget to add the base
            }
            var avg = 0;
            if (user_evals.length > 0) {
                avg = sum / user_evals.length;
            }
            avg = 2 * avg; // scale from 0-10
            var str = booking.bookingDate.split('T');
            var temp = str[0].split('-');
            var date = temp[2].toString() + '-' + temp[1].toString() + '-' + temp[0].toString();
            str[1] = str[1].split('.');
            var time =str[1][0];
            var bookingDate = date.toString() + " at " + time.toString();
            str = booking.startDate.split('T');
            temp = str[0].split('-');
            date = temp[2].toString() + '-' + temp[1].toString() + '-' + temp[0].toString();
            str[1] = str[1].split('.');
            time =str[1][0];
            var startDate = date.toString() + " at " + time.toString();
            str = booking.endDate.split('T');
            temp = str[0].split('-');
            date = temp[2].toString() + '-' + temp[1].toString() + '-' + temp[0].toString();
            str[1] = str[1].split('.');
            time =str[1][0];
            var endDate = date.toString() + " at " + time.toString();
            this.setState({
                avg: avg,
                bookingDate: bookingDate,
                startDate: startDate,
                endDate: endDate,
                rating: '',
                comment: ''
            });
        }).catch((e) => {
            console.error(e);
        });

    }

    handleChangeRating(event) {
        this.setState(Object.assign({}, this.state, {rating: event.target.value}));
    }

    handleChangeComment(event) {
        this.setState(Object.assign({}, this.state, {comment: event.target.value}));
    }

    handleSubmit(event) {
        console.log("in")
        event.preventDefault();
        let user_evaluation={};
        user_evaluation.rating = this.state.rating;
        user_evaluation.comment = this.state.comment;
        user_evaluation.evaluatee_id = this.props.booking.item_id.owner_id;
        console.log(user_evaluation)
        UserEvaluationService.createUserEvaluation(user_evaluation).then((data) => {
            this.props.history.push('/my_bookings')
        }).catch((e) => {
            console.error(e);
        });
    }

    render() {
        let booking = this.props.booking;
        console.log("in", booking)
        return (
            <Page>
                <Jumbotron>
                    <div class="row">
                            <div class="col">
                                <div class="row">
                                    <div className="mdc-card" style={{marginBottom:"2em", marginTop:"3em" ,marginLeft:"4em"} }>
                                        <div className="mdc-card__media mdc-card__media--square">
                                            <div className="mdc-card__media-content">
                                                <p>This booking concerns the following item:</p>
                                            </div>
                                            <div className="mdc-card__media-content">
                                                <SimpleLink className="card_link" to={`/my_item/${booking.item_id._id}`}>{booking.item_id.name}</SimpleLink>
                                            </div>
                                            <div className="mdc-card__media-content">
                                                <SimpleLink to={`/my_item/${booking.item_id._id}`}>
                                                    <img className="img-fluid" height={300} width={300} src = {`http://localhost:3000/photos/${booking.item_id.photos[0]}`}/>
                                                </SimpleLink>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                                <div class ="row">
                                    <div className="mdc-card" style={{marginBottom:"2em", marginLeft: "4em"}}>
                                        <div className="mdc-card__media mdc-card__media--square">
                                            <div className="mdc-card__media-content">
                                                <p>Information about the booker of your item!</p>
                                            </div>
                                            <div className="mdc-card__media-content">
                                                <SimpleLink className="card_link" to={`/profile/${booking.booker_id._id}`}>{booking.booker_id.username}</SimpleLink>
                                            </div>
                                            <div className="mdc-card__media-content">
                                                <SimpleLink to={`/profile/${booking.booker_id._id}`}>
                                                    <img className="img-fluid" height={300} width={300} src = {`http://localhost:3000/photos/${booking.booker_id.photo}`}/>
                                                </SimpleLink>
                                            </div>
                                        </div>
                                        <div className="mdc-card__media mdc-card__media--square custom_bdet">
                                            <StarRatingComponent
                                                name="rate1"
                                                starCount={10}
                                                value={this.state.avg}
                                            />
                                        </div>
                                    </div>
                                </div>
                            </div>
                            <div class="col" >
                                <div class="row">
                                    <Jumbotron style={{marginRight : "5em"}}>
                                        <h1 className="display-10">Here you can see more details about the booking of your item!</h1>
                                        <p className="lead"> {booking.item_id.name} was booked by {booking.booker_id.username} on {this.state.bookingDate} </p>
                                        <p className="lead">Starting Date: {this.state.startDate} </p>
                                        <p className="lead">End Date: {this.state.endDate} </p>
                                        <p className="lead">Price per day: {booking.item_id.price}</p>

                                        <hr className="my-2" />
                                        <p>Click on the user or the item to see more details about them!</p>
                                    </Jumbotron>
                                </div>
                            </div>
                        </div>
                </Jumbotron>
            </Page>
        );
    }

}

export default withRouter(MyItemBookingDetail);