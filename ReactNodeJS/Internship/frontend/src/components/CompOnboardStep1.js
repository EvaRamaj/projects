'use strict';

import React, {Component} from 'react';
import DatePicker from "react-md/lib/Pickers/DatePickerContainer";
import {SelectionControlGroup} from "react-md";

export default class CompOnboardStep1 extends Component {
    constructor(props) {
        super(props);

        this.state = {
            overview: '',
            frequency: '',
            startDate: '',
            endDate: '',
            travelling: '',
            location: ''
        };

        this.validationCheck = this.validationCheck.bind(this);

    }

    componentDidMount() {}

    componentWillUnmount() {}

    validationCheck() {
        if (!this._validateOnDemand)
            return;

        const userInput = this._grabUserInput(); // grab user entered vals
        const validateNewInput = CompOnboardStep1._validateData(userInput); // run the new input against the validator

        this.setState(Object.assign(userInput, validateNewInput, CompOnboardStep1._validationErrors(validateNewInput)));
    }

    static _validateData(data) {
        return {
            overviewVal: (data.overview !== 0), // required: anything besides N/A
            frequencyVal: (data.frequency !== 0),
            startDateVal: (data.startDate !== 0),
            endDateVal: (data.endDate !== 0),
            travellingVal: (data.travelling !== 0),
            locationVal: (data.location !==0)
        }
    }

    static _validationErrors(val) {
        return {
            overviewValMsg: val.overviewVal ? '' : 'An overview is required',
            frequencyValMsg: val.frequencyVal ? '' : 'The frequency is required',
            startDateValMsg: val.startDateVal ? '' : 'The startDate is required',
            endDateValMsg: val.endDateVal ? '' : 'The endDate is required',
            travellingValMsg: val.travellingVal ? '' : 'Travelling is required',
            locationValMsg:val.locationVal
        };
    }



    _grabUserInput() {
        return {
            overview: this.refs.overview.value,
            frequency: this.refs.frequency.value,
            startDate: this.refs.startDate.value,
            endDate: this.refs.endDate.value,
            travelling: this.refs.travelling.value,
            location: this.refs.location.value
        };
    }



    // not required as this component has no forms or user entry
    // isValidated() {}

    render() {

        let notValidClasses = {};

        if (typeof this.state.overviewVal === 'undefined' || this.state.overviewVal) {
            notValidClasses.overviewCls = 'no-error col-md-8';
        }
        else {
            notValidClasses.overviewCls = 'has-error col-md-8';
            notValidClasses.overviewValGrpCls = 'val-err-tooltip';
        }

        if (typeof this.state.frequencyVal === 'undefined' || this.state.frequencyVal) {
            notValidClasses.frequencyCls = 'no-error col-md-8';
        }
        else {
            notValidClasses.frequencyCls = 'has-error col-md-8';
            notValidClasses.frequencyValGrpCls = 'val-err-tooltip';
        }
        if (typeof this.state.startDateVal === 'undefined' || this.state.startDateVal) {
            notValidClasses.startDateCls = 'no-error col-md-8';
        }
        else {
            notValidClasses.startDateCls = 'has-error col-md-8';
            notValidClasses.startDateValGrpCls = 'val-err-tooltip';
        }
        if (typeof this.state.endDateVal === 'undefined' || this.state.endDateVal) {
            notValidClasses.endDateCls = 'no-error col-md-8';
        }
        else {
            notValidClasses.endDateCls = 'has-error col-md-8';
            notValidClasses.endDateValGrpCls = 'val-err-tooltip';
        }
        if (typeof this.state.travellingVal === 'undefined' || this.state.travellingVal) {
            notValidClasses.travellingCls = 'no-error col-md-8';
        }
        else {
            notValidClasses.travellingCls = 'has-error col-md-8';
            notValidClasses.travellingValGrpCls = 'val-err-tooltip';
        }
        if (typeof this.state.locationVal === 'undefined' || this.state.locationVal) {
            notValidClasses.locationCls = 'no-error col-md-8';
        }
        else {
            notValidClasses.locationCls = 'has-error col-md-8';
            notValidClasses.locationValGrpCls = 'val-err-tooltip';
        }

        return (
            <div className="step step1">
                <div className="row text-left" style={{paddingLeft:"40em", paddingRight:"40em"}}>
                    <form id="Form" className="form-row">
                        <div className="form-group">
                            <div className="text-left" >
                                <p style={{fontSize :"1em"}}>Tell us about the details of your project. This will be shown to selected experienced professionals. Please make sure that the details are as interesting and captivating as possible.</p>
                                <textarea style={{width:"100%"}}
                                    ref="overview"
                                    autoComplete="off"
                                    type="overview"
                                    placeholder="describe your project"
                                    className="form-control"
                                    required
                                    defaultValue={this.state.overview}
                                    onBlur={this.validationCheck} />
                                </div>
                            </div>
                            <div className="text-left">
                                <p style={{fontSize :"1em", marginTop:"4em"}}>Duration of the project:</p>
                                <DatePicker
                                    ref = "startDate"
                                    id="startDate"
                                    required
                                    label="From"
                                    defaultValue={this.state.startDate}
                                    onBlur={this.validationCheck}
                                    autoOk={true}/>
                                <DatePicker
                                    id="endDate"
                                    ref = "endDate"
                                    required
                                    label="To"
                                    defaultValue={this.state.endDate}
                                    onBlur={this.validationCheck}
                                    autoOk={true}/>
                            </div>
                            <div className="text-left" style={{marginTop:"4em", display: 'flex', flexDirection: 'row'}}>
                                <p style={{marginTop:"0.7em",fontSize :"1em", display: "inline",paddingRight:"1em"}}>Frequency:  </p>
                                <input style={{marginTop:"0.6em",height:"2em", display: 'flex', flexDirection: 'row',width:"100%"}}
                                       placeholder="How many times"
                                       ref = "frequency"
                                       className= "form-control"
                                       required
                                       label="Frequency"
                                       defaultValue={this.state.frequency}
                                       onBlur={this.validationCheck}

                                />
                                <SelectionControlGroup style={{display: 'flex', flexDirection: 'row'}}
                                    id="selection-control-group-radios"
                                    name="radio-frequency"
                                    className="md-row"
                                    type="radio"
                                    controls={[{
                                        label: 'Per Week',
                                        value: 'week'
                                    }, {
                                        label: 'Per Month',
                                        value: 'month'
                                    }]}
                                />
                            </div>
                            <div className="text-left" style={{marginTop:"5em", display: 'flex', flexDirection: 'row'}}>
                                <p style={{marginTop:"0.7em",fontSize :"1em", display: "inline", paddingRight:"1em"}}>Location: </p>
                                <input style={{marginTop:"0.6em",height:"2em", display: 'flex', flexDirection: 'row',width:"100%"}}
                                       placeholder="Region"
                                       ref = "location"
                                       className= "form-control"
                                       required
                                       label="Location"
                                       defaultValue={this.state.location}
                                       onBlur={this.validationCheck}
                                />
                            </div>
                            <div className="text-left" style={{marginTop:"5em", display: 'flex', flexDirection: 'row'}}>
                                <p style={{marginTop:"0.7em",fontSize :"1em", display: "inline"}}>Whould you like to travel?  </p>
                                <SelectionControlGroup style={{display: 'flex', flexDirection: 'row', width:"100%"}}
                                                       id="travelling"
                                                       name="radio-travelling"
                                                       className="md-row"
                                                       type="radio"
                                                       controls={[{
                                                           label: 'Yes',
                                                           value: 'yes'
                                                       }, {
                                                           label: 'No',
                                                           value: 'no'
                                                       }, {
                                                           label: 'Only inside the country',
                                                           value: 'maybe'
                                                       }
                                                       ]}
                                />
                            </div>
                    </form>
                </div>
            </div>
        )
    }
}
