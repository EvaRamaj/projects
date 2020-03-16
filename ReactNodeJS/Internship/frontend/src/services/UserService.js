"use strict";

import HttpService from "./HttpService";

export default class UserService {

    constructor() {
    }

    static baseURL() {return HttpService.apiURL()+"/auth"; }
    static baseURLProfile() {return HttpService.apiURL()+"/user"; }

    static register(user) {
        return new Promise((resolve, reject) => {
            HttpService.post(`${UserService.baseURL()}/register`, user, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static login(user, pass, role) {
        return new Promise((resolve, reject) => {
            HttpService.post(`${UserService.baseURL()}/login`, {
                username: user,
                password: pass,
                role: role
            }, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static logout(){
        window.localStorage.removeItem('jwtToken');
    }

    static getCurrentUser() {
        let token = window.localStorage['jwtToken'];
        if (!token) return {};

        let base64Url = token.split('.')[1];
        let base64 = base64Url.replace('-', '+').replace('_', '/');
        return {
            id : JSON.parse(window.atob(base64)).id,
            username: JSON.parse(window.atob(base64)).username,
            role: JSON.parse(window.atob(base64)).role
        };
    }

    static isAuthenticated() {
        return !!window.localStorage['jwtToken'];
    }
    static requestUpgrade(user) {
        return new Promise((resolve, reject) => {
            HttpService.put(`${this.baseURLProfile()}/${user.id}`, user, function (data) {
                resolve(data);
            }, function (textStatus) {
                reject(textStatus);
            });
        });

    }
    static getUser(id) {
        return new Promise((resolve, reject) => {
            HttpService.get(`${this.baseURLProfile()}/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    resolve(data);
                }
                else {
                    reject('malakia_1 : We cannot get the user');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getCurrentUserDetails()
    {
        const userCreds = this.getCurrentUser();
        if (userCreds.id != null)
        {
            return new Promise((resolve, reject) => {
                HttpService.get(`${this.baseURLProfile()}/${userCreds.id}`, function(data) {
                    resolve(data);
                }, function(textStatus) {
                    reject(textStatus);
                });
            });
        }
        else {
            reject("User is not logged In");
        }
    }

    static linkedIn(auth_code,url,role)
    {
        return new Promise((resolve, reject) => {
            HttpService.postLinkedin(`${this.baseURL()}/${url}`, {code: auth_code, role: role}, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static confirmEmail(token) {
        return new Promise((resolve, reject) => {
            HttpService.put(`${UserService.baseURL()}/confirmEmail`, {token: token}, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getUserCount() {
        return new Promise((resolve, reject) => {
            HttpService.get(`${this.baseURLProfile()}/count`, function(data) {
                    resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
}
