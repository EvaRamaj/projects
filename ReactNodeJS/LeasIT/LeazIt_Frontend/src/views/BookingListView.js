"use strict";

import React from 'react';

import { BookingList } from '../components/BookingList';

import BookingService from '../services/BookingService';
import ItemService from '../services/ItemService';
// let dateFormat = require('dateformat');

export class BookingListView extends React.Component {

    constructor(props) {
        super(props);
        console.log('BookingListView', props);
        this.state = {
            loading: false,
            data: [],
        };
    }

    componentWillMount(){
        this.setState({
            loading: true
        });
        if(this.props.match.path === '/my_bookings') {
            BookingService.getBookings().then((data) => {
                console.log("in3",data)
                let bookingItems = data.bookingItems;
                // bookingItems.startDate = dateFormat(bookingItems.startDate, "dddd, mmmm dS, yyyy");
                // console.log(dateFormat(bookingItems.startDate, "dddd mmmm dS yyyy"));
                this.setState({
                    data: [...data.bookingItems],
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });
        }
        else{
            let id = this.props.match.params.id;
            console.log('id:', id);
            BookingService.getItemBookings(id).then((data) => {
                console.log("salala",data)
                this.setState({
                    data: [...data.bookingItems],
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });

        }

    }

    deleteBooking(id) {
        this.setState({
            data: [...this.state.data],
            loading: true
        });
        BookingService.deleteBooking(id).then((message) => {

            let bookingIndex = this.state.data.map(booking => booking['_id']).indexOf(id);
            let bookings = this.state.data;
            bookings.splice(bookingIndex, 1);
            this.setState({
                data: [...bookings],
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    onBookingDetails(id){
        let url = `/booking/${id}`;
        console.log(url);
        this.props.history.push(url);
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        return (
            <BookingList data={this.state.data} match={this.props.match} itemId={this.props.match.params.id} onDelete={(id) => this.deleteBooking(id)} onClick={(id) => this.onBookingDetails(id)}/>
        );
    }
}
