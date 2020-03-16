"use strict";

import React from 'react';
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon } from 'react-md';
import { Jumbotron } from 'reactstrap';
import { Form, FormGroup, Label, Input } from 'reactstrap';
import { SimpleLink } from './SimpleLink';
import StarRatingComponent from "react-star-rating-component"
import  MapContainer  from "./Map";

import Page from './Page';

import UserEvaluationService from '../services/UserEvaluationService';
import UserService from '../services/UserService';
import {withRouter} from "react-router-dom";

class BookingDetail extends React.Component {

    constructor(props) {
        super(props);
        this.state ={
            loading:true,
            avg: 0,
            bookingDate:'',
            startDate:'',
            endDate:'',
            rating: '',
            comment: '',
            user:undefined
        }
        this.handleChangeRating = this.handleChangeRating.bind(this);
        this.handleChangeComment = this.handleChangeComment.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);

    }
    componentWillMount() {
        let booking = this.props.booking;
        console.log("in", booking.item_id.owner_id);
        UserEvaluationService.getUserEvaluation(booking.item_id.owner_id).then((user_evals) => {
            var sum = 0;
            for (var i = 0; i < user_evals.evaluations.length; i++) {
                sum += user_evals.evaluations[i].rating; //don't forget to add the base
            }
            var avg = 0;
            if (user_evals.evaluations.length > 0) {
                avg = sum / user_evals.evaluations.length;
            }
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
            time =str[1][0];
            var endDate = date.toString() + " at " + time.toString();
            this.setState({
                avg: avg,
                bookingDate: bookingDate,
                startDate: startDate,
                endDate: endDate,
                rating: '',
                comment: '',
                user:undefined
            });
            UserService.getUser(booking.item_id.owner_id).then((user) => {
                this.setState({
                    avg: avg,
                    bookingDate: bookingDate,
                    startDate: startDate,
                    endDate: endDate,
                    rating: '',
                    comment: '',
                    user: user,
                    loading: false
                })
            }).catch((e) => {
                console.error(e);
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    handleChangeRating(value) {
        this.setState(Object.assign({}, this.state, {rating: value}));
    }

    handleChangeComment(event) {
        this.setState(Object.assign({}, this.state, {comment: event.target.value}));
    }

    handleSubmit(event) {
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
        if(this.state.loading){
            return(
              <h2>Loading....</h2>
            );
        }
        let booking = this.props.booking;
        return (
            <Page>
                <div class="row">
                    <div class="col">
                        <div class="row">
                            <div className="mdc-card" style={{marginBottom:"2em", marginRight: "2em", marginTop:"1em"}}>
                                <div className="mdc-card__media mdc-card__media--square">
                                    <div className="mdc-card__media-content">
                                        <p>You booked the following item:</p>
                                    </div>
                                    <div className="mdc-card__media-content">
                                        <SimpleLink className="card_link" to={`/item/${booking.item_id._id}`}>{booking.item_id.name}</SimpleLink>
                                    </div>
                                    <div className="mdc-card__media-content">
                                        <SimpleLink to={`/item/${booking.item_id._id}`}>
                                            <img className="img-fluid" height={300} width={300} src = {`http://localhost:3000/photos/${booking.item_id.photos[0]}`}/>
                                        </SimpleLink>
                                    </div>
                                </div>
                            </div>
                        </div>
                        <div class ="row">
                            <div className="mdc-card" style={{marginBottom:"2em", marginRight: "2em"}}>
                                <div className="mdc-card__media mdc-card__media--square">
                                    <div className="mdc-card__media-content">
                                        <p>Information about the owner of your booked item!</p>
                                    </div>
                                    <div className="mdc-card__media-content">
                                        <SimpleLink className="card_link" to={`/profile/${booking.item_id.owner_id}`}>{this.state.user.username}</SimpleLink>
                                    </div>
                                    <div className="mdc-card__media-content">
                                        <SimpleLink to={`/profile/${booking.booker_id._id}`}>
                                            <img className="img-fluid" height={300} width={300} src = {`http://localhost:3000/photos/${this.state.user.photo}`}/>
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
                    <div class="col">
                        <div class="row">
                            <Jumbotron style={{marginTop : "3em" ,marginLeft: "1em"}}>
                                <h1 className="display-10">Here you can see more details about your booking!</h1>
                                <p className="lead"> You booked this item on {this.state.bookingDate} </p>
                                <p className="lead">Starting Date: {this.state.startDate} </p>
                                <p className="lead">End Date: {this.state.endDate} </p>
                                <hr className="my-2" />
                                <p>Click on the user or the item to see more details about them!</p>
                            </Jumbotron>
                        </div>
                    </div>
                    <div className="col">
                        <div className="row" style={{marginLeft:"5em", marginTop:"3em"}}>
                            <MapContainer google={this.props.google} coord={this.state.user.coordinates}/>
                        </div>
                        <div className="row" style={{marginLeft:"5em", marginTop:"14em"}}>
                            <Jumbotron>
                                <p>Here you can rate your overall experience!</p>
                                <Form onSubmit={this.handleSubmit}>
                                    <FormGroup>
                                        <Label for="exampleEmail">Comment</Label>
                                        <Input onChange={this.handleChangeComment} type="textarea" name="comment" id="comment" />
                                    </FormGroup>
                                    <FormGroup style={{marginLeft: "2.5em"}}>
                                        <StarRatingComponent
                                            name="rate1"
                                            starCount={10}
                                            onStarClick={this.handleChangeRating}
                                            value={this.state.rating}
                                        />
                                    </FormGroup>
                                    <Button style={{marginLeft: "1em"}} type="submit" >Submit</Button>
                                </Form>
                            </Jumbotron>
                        </div>
                    </div>
                </div>
            </Page>
        );
    }

}

export default withRouter(BookingDetail);