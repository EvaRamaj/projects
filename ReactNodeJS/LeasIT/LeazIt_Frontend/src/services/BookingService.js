"use strict";

import HttpService from './HttpService';

export default class BookingService {

    constructor(){
    }

    static baseURL() {return "http://localhost:3000/bookings" }

    static createBooking(booking,itemId) {
        console.log(booking);
        return new Promise((resolve, reject) => {
            HttpService.post(`${BookingService.baseURL()}/${itemId}`, booking, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getBookings(){
        return new Promise((resolve, reject) => {
            HttpService.get(this.baseURL(), function(data) {
                console.log('getBookings', data);
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getItemBookings(id){
        return new Promise((resolve, reject) => {
            HttpService.get(`${BookingService.baseURL()}/${id}`, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
    static getItemBooking(id,itemId){
        return new Promise((resolve, reject) => {
            HttpService.get(`${BookingService.baseURL()}/${itemId}/details/${id}`, function(data) {
                console.log(data);
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getBooking(id) {

        console.log(`${BookingService.baseURL()}/details/${id}`);

        return new Promise((resolve, reject) => {
            HttpService.get(`${BookingService.baseURL()}/details/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    console.log('booking details data :', data);
                    resolve(data);
                }
                else {
                    reject('Error while retrieving booking details');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static deleteBooking(id) {
        return new Promise((resolve, reject) => {
            HttpService.remove(`${BookingService.baseURL()}/${id}`, function(data) {
                if(data.message != undefined) {
                    resolve(data.message);
                }
                else {
                    reject('Error while deleting');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

}