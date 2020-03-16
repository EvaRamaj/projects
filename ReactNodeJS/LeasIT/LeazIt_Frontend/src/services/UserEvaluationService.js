"use strict";

import HttpService from './HttpService';

export default class UserEvaluationService {

    constructor(){
    }

    static baseURL() {return "http://localhost:3000/user_evaluations" }

    static getUserEvaluations(){
        return new Promise((resolve, reject) => {
            HttpService.get(this.baseURL(), function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getUserEvaluation(id) {
        console.log(`${this.baseURL()}/${id}`)
        return new Promise((resolve, reject) => {
            HttpService.get(`${this.baseURL()}/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    resolve(data);
                }
                else {
                    reject('Error while retrieving user evaluation');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }


    static getEvaluationDetails(id) {

        return new Promise((resolve, reject) => {
            HttpService.get(`${UserEvaluationService.baseURL()}/user_evaluation/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    console.log('user evaluation details data :', data);
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

    static deleteUserEvaluation(id) {
        return new Promise((resolve, reject) => {
            HttpService.remove(`${UserEvaluationService.baseURL()}/${id}`, function(data) {
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


    static createUserEvaluation(user_evaluation) {
        console.log("inin",user_evaluation);

        return new Promise((resolve, reject) => {
            HttpService.post(`${UserEvaluationService.baseURL()}/${user_evaluation.evaluatee_id}`, user_evaluation, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
}