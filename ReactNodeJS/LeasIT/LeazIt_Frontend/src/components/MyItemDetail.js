"use strict";

import React from 'react';
import { Link, withRouter } from 'react-router-dom'
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon, TextField } from 'react-md';
import { SimpleLink } from './SimpleLink';
import { Jumbotron } from 'reactstrap';
import { ListGroup, ListGroupItem } from 'reactstrap';
import  MapContainer  from "./Map";
import Page from './Page';
import StarRatingComponent from "react-star-rating-component"

import AuthService from '../services/AuthService';
import UserEvaluationService from '../services/UserEvaluationService';
import BookingService from '../services/BookingService';

class MyItemDetail extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            startDate: '',
            endDate: '',
            loading: true
        }
        this.handleChangeStartDate = this.handleChangeStartDate.bind(this);
        this.handleChangeEndDate = this.handleChangeEndDate.bind(this);

        this.handleSubmit = this.handleSubmit.bind(this);
    }

    componentWillMount(props){
        UserEvaluationService.getUserEvaluation(this.props.item.owner_id._id).then((user_evals) => {
            var sum = 0;
            for (var i = 0; i < user_evals.evaluations.length; i++) {
                sum += user_evals.evaluations[i].rating; //don't forget to add the base
            }
            var avg = 0;
            if (user_evals.evaluations.length > 0) {
                avg = sum / user_evals.evaluations.length;
            }
            this.setState({
                avg: avg,
                user_evals:user_evals,
                loading: false
            });
        });
    };

    handleChangeStartDate(value) {
        this.setState(Object.assign({}, this.state, {startDate: value}));

    }

    handleChangeEndDate(value) {
        this.setState(Object.assign({}, this.state, {endDate: value}));
    }

    handleSubmit(event){
        if(AuthService.isAuthenticated())
        {
            var itemId = this.props.item._id;
            var booking={
               startDate: this.state.startDate,
               endDate: this.state.endDate
            };
            BookingService.createBooking(booking,itemId).then((data) => {
                this.props.history.push('/my_bookings');
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
            });
        }
        else{
            this.props.history.push('/register');
        }
    }
    render() {
        if(this.state.loading){
            return(
                <h2>Loading....</h2>
            );
        }
        let item = this.props.item;
        let user_evals = this.state.user_evals;
        console.log("in10000",item.owner_id.coordinates);
        return (
            <Page>
                <div class="row">
                    <div class="col-xs">
                        <div class="row-xs">
                            <div className="mdc-card" style={{marginBottom:"2em", marginTop:"1em"}}>
                                <div className="mdc-card__media mdc-card__media--square">
                                    <div className="mdc-card__media-content">
                                        <SimpleLink className="card_link" to={`/my_item/${item._id}`}>{item.name}</SimpleLink>
                                    </div>
                                    <div className="mdc-card__media-content">
                                        <SimpleLink to={`/my_item/${item._id}`}>
                                            <img className="img-fluid" height={300} width={300} src = {`http://localhost:3000/photos/${item.photos[0]}`}/>
                                        </SimpleLink>
                                    </div>
                                    <div className="mdc-card__media mdc-card__media--square custom">
                                        <StarRatingComponent
                                            name="rate1"
                                            starCount={10}
                                            value={this.state.avg}
                                        />
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                    <div class="col">
                        <div class="row">
                            <Jumbotron style={{marginTop : "3em" ,marginLeft: "3em"}}>
                                <h1 className="display-10">Here you can see more details about your item!</h1>
                                <p className="lead"> {item.name} </p>
                                <p className="lead">{item.description} </p>
                                <hr className="my-2" />
                                <p className="lead"> {item.price} <i className="fa fa-euro"></i> per day </p>
                                <hr className="my-2" />
                                <div className="mdc-card__action-buttons custom">
                                    {AuthService.isAuthenticated() ?
                                        <SimpleLink className = "card_link" to={`/edit/${item._id}`}><FontIcon>mode_edit</FontIcon></SimpleLink>
                                        :
                                        <SimpleLink to={'/login'} className="card_link"><FontIcon>mode_edit</FontIcon></SimpleLink>
                                    }
                                </div>
                                <div className="mdc-card__action-icons">
                                    {AuthService.isAuthenticated() ?
                                        <button className="button_list" onClick={() => this.props.onDelete(item._id)} icon>delete</button>
                                        :
                                        <Link to={'/login'}><FontIcon>delete</FontIcon></Link>
                                    }
                                </div>
                            </Jumbotron>
                        </div>
                    </div>
                    <div class="col">
                        <div class="row">
                            <div className="row" style={{marginLeft:"5em", marginTop:"3em"}}>
                                <MapContainer google={this.props.google} coord={item.owner_id.coordinates}/>
                            </div>
                            <div class="row" style={{marginLeft:"1em", marginTop:"18em"}}>
                                <div className="mdc-card__media-content" style={{maxHeight:"380px", width:"285px", overflow:"auto"}}>
                                    <p className="list_item">Evaluation Details</p>
                                    <ListGroup>
                                        {user_evals.evaluations.map((user_eval, i) => <ListGroup className="list_item">
                                                <ListGroupItem className="list_item">
                                                    Evaluator:
                                                    <SimpleLink to={`/profile/${user_eval.evaluator_id._id}`}>
                                                        {user_eval.evaluator_id.username}
                                                    </SimpleLink>

                                                    </ListGroupItem>
                                                <ListGroupItem className="list_item">Comment: {user_eval.comment} </ListGroupItem>
                                                <ListGroupItem className="list_item">
                                                    <StarRatingComponent
                                                        name="rate1"
                                                        starCount={10}
                                                        value={user_eval.rating}
                                                    />
                                                </ListGroupItem>
                                            </ListGroup>)}
                                    </ListGroup>
                                </div>
                            </div>
                        </div>
                    </div>

                </div>
            </Page>
        );
    }
}
export default withRouter(MyItemDetail)