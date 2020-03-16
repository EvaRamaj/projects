'use strict';

import React, { Component } from 'react';


export default class CompOnboardStep2 extends Component {
    constructor(props) {
        super(props);

        this.state = {
            responsibilities: '',
            role: '',
            language: ''
        };

        this.validationCheck = this.validationCheck.bind(this);

    }
    validationCheck() {
        if (!this._validateOnDemand)
            return;

        const userInput = this._grabUserInput(); // grab user entered vals
        const validateNewInput = CompOnboardStep2._validateData(userInput); // run the new input against the validator

        this.setState(Object.assign(userInput, validateNewInput, CompOnboardStep2._validationErrors(validateNewInput)));
    }

    static _validateData(data) {
        return {
            responsibilitiesVal: (data.responsibilities !== 0), // required: anything besides N/A
            roleVal: (data.role !== 0),
            languageVal: (data.language !== 0),
        }
    }

    static _validationErrors(val) {
        return {
            responsibilitiesValMsg: val.responsibilitiesVal ? '' : 'An overview is required',
            roleValMsg: val.roleVal ? '' : 'The frequency is required',
            languageValMsg: val.languageVal ? '' : 'The startDate is required',
        };
    }



    _grabUserInput() {
        return {
            responsibilities: this.refs.responsibilities.value,
            role: this.refs.role.value,
            language: this.refs.language.value,
        };
    }


    componentDidMount() {}

    componentWillUnmount() {}


    render() {
        let notValidClasses = {};

        if (typeof this.state.responsibilitiesVal === 'undefined' || this.state.responsibilitiesVal) {
            notValidClasses.responsibilitiesCls = 'no-error col-md-12';
        }
        else {
            notValidClasses.responsibilitiesCls = 'has-error col-md-12';
            notValidClasses.responsibilitiesValGrpCls = 'val-err-tooltip';
        }

        if (typeof this.state.roleVal === 'undefined' || this.state.roleVal) {
            notValidClasses.roleCls = 'no-error col-md-12';
        }
        else {
            notValidClasses.roleCls = 'has-error col-md-12';
            notValidClasses.roleValGrpCls = 'val-err-tooltip';
        }
        if (typeof this.state.languageVal === 'undefined' || this.state.languageVal) {
            notValidClasses.languageCls = 'no-error col-md-12';
        }
        else {
            notValidClasses.languageCls = 'has-error col-md-12';
            notValidClasses.languageValGrpCls = 'val-err-tooltip';
        }

        return (
            <div className="step step3">
                <div className="row text-center" style={{paddingLeft:"40em", paddingRight:"40em"}}>
                    <form id="Form" className="form-row">
                        <div className="text-left" >
                            <div className={notValidClasses.responsibilitiesCls}>
                                <p style={{fontSize :"1em", marginTop:"4em"}}>Tell us about the responsibilities that the employee should have for this project: </p>
                                <textarea style={{width:"100%"}}
                                          ref="responsibilities"
                                          autoComplete="off"
                                          type="responsibilities"
                                          placeholder="Define the responsibilities"
                                          className="form-control"
                                          required
                                          defaultValue={this.state.responsibilities}
                                          onBlur={this.validationCheck} />
                                <div className={notValidClasses.responsibilitiesValGrpCls}>{this.state.responsibilitiesValMsg}</div>
                            </div>
                        </div>
                        <div className="text-left" >
                            <div className={notValidClasses.roleCls}>
                                <p style={{marginTop:"4em",fontSize :"1em",paddingRight:"1em"}}>Role:  </p>
                                <input style={{marginTop:"0.6em",height:"2em", display: 'flex', flexDirection: 'row',width:"100%"}}
                                       placeholder="Role"
                                       ref = "role"
                                       className= "form-control"
                                       required
                                       label="Role"
                                       defaultValue={this.state.role}
                                       onBlur={this.validationCheck}
                                />
                                <div className={notValidClasses.roleValGrpCls}>{this.state.roleValMsg}</div>
                            </div>
                        </div>
                        <div className="text-left" >
                            <div className={notValidClasses.languageCls}>
                                <p style={{marginTop:"4em",fontSize :"1em",paddingRight:"1em"}}>Languages that are needed for this project:  </p>
                                <input style={{marginTop:"0.6em",height:"2em", display: 'flex', flexDirection: 'row',width:"100%"}}
                                       placeholder="Languages"
                                       ref = "role"
                                       className= "form-control"
                                       required
                                       label="Languages"
                                       defaultValue={this.state.language}
                                       onBlur={this.validationCheck}
                                />
                                <div className={notValidClasses.languageValGrpCls}>{this.state.languageValMsg}</div>
                            </div>
                        </div>
                    </form>
                </div>
            </div>

        )
    }
}

