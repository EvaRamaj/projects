"use strict";

import HttpService from "./HttpService";

export default class AuthService {

    constructor() {
    }

    static baseURL() {return "http://localhost:3000/auth"; }

    static register(user) {
        return new Promise((resolve, reject) => {
            HttpService.post(`${AuthService.baseURL()}/register`, {
                username : user.username,
                password : user.password,
                email : user.email,
                first_name : user.first_name,
                last_name : user.last_name,
                address : user.address,
                phone : user.phone,
                photo : user.photo,
                id_document : user.id_document,
                lessor_role_requested : user.lesor_role_requested,
                coordinates: user.coordinates

            }, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static login(user, pass) {
        return new Promise((resolve, reject) => {
            HttpService.post(`${AuthService.baseURL()}/login`, {
                username: user,
                password: pass
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
        // console.log(token)
        if (!token) return {};

        let base64Url = token.split('.')[1];
        let base64 = base64Url.replace('-', '+').replace('_', '/');
        console.log("tok",base64);
        var photo= JSON.parse(window.atob(base64))
        console.log(photo)
        return {
            id : JSON.parse(window.atob(base64)).id,
            username: JSON.parse(window.atob(base64)).username,
            photo: JSON.parse(window.atob(base64)).photo
        };
    }

    static isAuthenticated() {
        return !!window.localStorage['jwtToken'];
    }

    static getRole() {
        if(!!window.localStorage['jwtToken']){
            return window.localStorage['role']
        }
        return undefined;
    }
}