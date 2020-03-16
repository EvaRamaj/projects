"use strict";

import React from 'react';

import BookingDetail  from '../components/BookingDetail';
import MyItemBookingDetail  from '../components/MyItemBookingDetail';

import BookingService from '../services/BookingService';


export class BookingDetailView extends React.Component {

    constructor(props) {
        super(props);
        this.state={
            loading: true,
            booking: [],
            item: undefined
        }
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });
        let id = this.props.match.params.id;
        console.log(id);
        if(this.props.match.path === '/booking/:id') {
            BookingService.getBooking(id).then((data) => {
                console.log('BookingDetailView:', data);
                this.setState({
                    item: 0,
                    booking: data.booking.booking,
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });
        }
        else{
            let itemId = this.props.match.params.itemId;
            BookingService.getItemBooking(id,itemId).then((data) => {
                this.setState({
                    item: 1,
                    booking: data.bookingItems,
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });
        }
    }

    deleteBooking(id) {
        BookingService.deleteBooking(id).then((message) => {
            this.props.history.push('/');
        }).catch((e) => {
            console.log(e);
        });
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        if(this.state.item){
            return (
                <MyItemBookingDetail booking={this.state.booking[0]} onDelete={(id) => this.deleteBooking(id)}/>
            );
        }
        else{
            return (
                <BookingDetail booking={this.state.booking[0]} onDelete={(id) => this.deleteBooking(id)}/>
            );
        }
    }
}
