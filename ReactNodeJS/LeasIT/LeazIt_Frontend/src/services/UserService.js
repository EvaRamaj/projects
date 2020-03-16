"use strict";

import HttpService from './HttpService';

export default class UserService {

    constructor(){
    }

    static baseURL() {return "http://localhost:3000/users" }

    static getUsers(){
        return new Promise((resolve, reject) => {
            HttpService.get(this.baseURL(), function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getLessorRequests(){
        return new Promise((resolve, reject) => {
            HttpService.get(`${UserService.baseURL()}/admin/lessor_requests`, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }



    static getUser(id) {
        return new Promise((resolve, reject) => {
            HttpService.get(`${UserService.baseURL()}/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    // console.log('data :', data);
                    resolve(data);
                }
                else {
                    reject('Error while retrieving a user');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getUserByUsername(username) {
        return new Promise((resolve, reject) => {
            // console.log("in9",id);
            HttpService.get(`${UserService.baseURL()}/getByUsername/${username}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    // console.log('data :', data);
                    resolve(data);
                }
                else {
                    reject('Error while retrieving a user');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getMyProfile() {
        return new Promise((resolve, reject) => {
            HttpService.get(`${UserService.baseURL()}/profile`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    resolve(data);
                }
                else {
                    reject('Error while retrieving a user');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static deleteUser(id) {
        return new Promise((resolve, reject) => {
            HttpService.remove(`${UserService.baseURL()}/${id}`, function(data) {
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

    static deleteMyProfile() {
        return new Promise((resolve, reject) => {
            HttpService.remove(`${UserService.baseURL()}/profile/delete`, function(data) {
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

    static grantLessorPrivileges(user) {
        return new Promise((resolve, reject) => {
            HttpService.put(`${this.baseURL()}/admin/lessor_requests/${user._id}`, user, function (data) {
                console.log(data);
                resolve(data);
            }, function (textStatus) {
                reject(textStatus);
            });
        });
    }
    static requestUpgrade(user) {
        return new Promise((resolve, reject) => {
            HttpService.put(`${this.baseURL()}/profile/request`, user, function (data) {
                resolve(data);
            }, function (textStatus) {
                reject(textStatus);
            });
        });
    }

    static updateUser(user) {
        return new Promise((resolve, reject) => {
            HttpService.put(`${this.baseURL()}/profile/edit`, user, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
}