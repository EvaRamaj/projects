"use strict";

import React from 'react';
import { Link, withRouter } from 'react-router-dom'
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon, TextField } from 'react-md';

import Page from './Page';

import AuthService from '../services/AuthService';
import BookingService from '../services/BookingService';

class ItemDetail extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            startDate: '',
            endDate: ''
        }
        this.handleChangeStartDate = this.handleChangeStartDate.bind(this);
        this.handleChangeEndDate = this.handleChangeEndDate.bind(this);

        this.handleSubmit = this.handleSubmit.bind(this);
    }

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
            console.log("mesaaa")
            this.props.history.push('/register');
        }
    }
    render() {
        let item = this.props.item;
        console.log("in4",item)
        return (
            <Page>
                <Card>
                    <Grid className="grid-example" >
                        <Cell size={3}>
                            <CardText>
                                {item['photos'].map(function(key,index) {
                                        return <Media >
                                            <img src={`http://localhost:3000/photos/${item['photos'][index]}`} alt={item['name']} />
                                        </Media>
                                })}
                            </CardText>
                        </Cell>
                        <Cell size={7}/>
                    </Grid>
                    <CardText>
                        {Object.keys(item).map(function(key) {
                            if(key !=='photos'){
                                if(key === 'categories'){
                                     return <div>{key}: {item[key][0].name}</div>
                                 }
                                if(key === 'owner_id'){
                                    return <div>{key}: {item[key].username}</div>
                                }
                                return <div>{key}: {item[key]}</div>

                            }

                        })}
                    </CardText>
                </Card>
                <form className="form-inline my-2 my-lg-0" onSubmit={this.handleSubmit}>
                    <TextField
                        label="startDate"
                        id="SynopsisField"
                        type="date"
                        className="md-row"
                        required={true}
                        value={this.state.startDate}
                        onChange={this.handleChangeStartDate}
                        errorText="Synopsis is required"/>
                    <TextField
                        label="endDate"
                        id="SynopsisField"
                        type="date"
                        className="md-row"
                        required={true}
                        value={this.state.endDate}
                        onChange={this.handleChangeEndDate}
                        errorText="Synopsis is required"/>
                    <Button className="btn btn-outline-success my-2 my-sm-0" type="submit">LeaseIt!</Button>
                </form>
            </Page>
        );
    }
}
export default withRouter(ItemDetail)