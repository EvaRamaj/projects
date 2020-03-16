"use strict";

import HttpService from './HttpService';

export default class CategoryService {

    constructor(){
    }

    static baseURL() {return "http://localhost:3000/categories" }

    static getCategories(){
       return new Promise((resolve, reject) => {
           HttpService.get(this.baseURL(), function(data) {
               resolve(data);
           }, function(textStatus) {
               reject(textStatus);
           });
       });
    }

    static getCategory(id) {
        return new Promise((resolve, reject) => {
            HttpService.get(`${CategoryService.baseURL()}/${id}`, function(data) {
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

    static deleteCategory(id) {
        return new Promise((resolve, reject) => {
            HttpService.remove(`${CategoryService.baseURL()}/${id}`, function(data) {
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

    static updateCategory(category) {
        console.log(category);
        return new Promise((resolve, reject) => {
            HttpService.put(`${this.baseURL()}/${category._id}`, category, function(data) {
                resolve(data);
            }, function(textStatus) {
               reject(textStatus);
            });
        });
    }

    static createCategory(category) {
        return new Promise((resolve, reject) => {
            HttpService.post(CategoryService.baseURL(), category, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
}