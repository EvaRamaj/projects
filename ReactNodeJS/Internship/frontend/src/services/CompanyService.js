"use strict";

import HttpService from "./HttpService";

export default class CompanyService {

    constructor() {
    }

    static baseURL() {return HttpService.apiURL()+"/company"; }

    static search(query){
        return new Promise((resolve, reject) => {
            HttpService.post(`${CompanyService.baseURL()}/search`, query , function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static sendMail(data){
        return new Promise((resolve, reject) => {
            HttpService.post(`${CompanyService.baseURL()}/sendMail`, data , function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static register(company) {
        return new Promise((resolve, reject) => {
            HttpService.post(`${CompanyService.baseURL()}/create`, {
                username: company.email,
                password: company.password,
                profileData: company.profileData
            }, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static login(user, pass) {
        return new Promise((resolve, reject) => {
            HttpService.post(`${CompanyService.baseURL()}/login`, {
                username: user,
                password: pass
            }, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static isAuthenticated() {
        return !!window.localStorage['jwtToken'];
    }
}
