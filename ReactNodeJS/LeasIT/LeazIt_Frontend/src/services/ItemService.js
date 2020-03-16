"use strict";

import HttpService from './HttpService';

export default class ItemService {

    constructor(){
    }

    static baseURL() {return "http://localhost:3000/items" }

    static getItems(){
       return new Promise((resolve, reject) => {
           HttpService.get(this.baseURL(), function(data) {
               resolve(data);
           }, function(textStatus) {
               reject(textStatus);
           });
       });
    }

    static getMyItems(){
       return new Promise((resolve, reject) => {
           HttpService.get(`${ItemService.baseURL()}/my_items`, function(data) {
               resolve(data);
           }, function(textStatus) {
               reject(textStatus);
           });
       });
    }

    static getItem(id) {
        return new Promise((resolve, reject) => {
            HttpService.get(`${ItemService.baseURL()}/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    resolve(data);
                }
                else {
                    reject('Error while retrieving movie');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static deleteItem(id) {
        return new Promise((resolve, reject) => {
            HttpService.remove(`${ItemService.baseURL()}/${id}`, function(data) {
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

    static updateItem(item) {
        console.log(item);
        return new Promise((resolve, reject) => {
            HttpService.put(`${this.baseURL()}/${item._id}`, item, function(data) {
                resolve(data);
            }, function(textStatus) {
               reject(textStatus);
            });
        });
    }

    static createItem(item) {
        return new Promise((resolve, reject) => {
            HttpService.post(ItemService.baseURL(), item, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
    static searchItem(regular_expression) {
        let regular_expression_json = {q: regular_expression};
        console.log("json",`${ItemService.baseURL()}/search`);
        return new Promise((resolve, reject) => {
            HttpService.post(`${ItemService.baseURL()}/search`, regular_expression_json, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
}