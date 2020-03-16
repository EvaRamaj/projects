"use strict";

import React from 'react';
import {Button, TextField, Subheader, DatePicker} from 'react-md';
import { withRouter } from 'react-router-dom';
import { AlertMessage } from './AlertMessage';
import Page from './Page';
import UserService from "../services/UserService";

class NewProject extends React.Component
{
    constructor(props) {
        super(props);
        this.state = {
            user: UserService.isAuthenticated() ? UserService.getCurrentUser() : undefined,
            title: "",
            startDate: "",
            endDate: "",
            location: "",
            teamSetup: "",
            experience: "",
            travelling: "",
            overview: "",
            responsibilities: ""
        };

        this.handleChangeTitle = this.handleChangeTitle.bind(this);
        this.handleChangeLocation = this.handleChangeLocation.bind(this);
        this.handleChangeStartDate = this.handleChangeStartDate.bind(this);
        this.handleChangeEndDate = this.handleChangeEndDate.bind(this);
        this.handleChangeTeamSetup = this.handleChangeTeamSetup.bind(this);
        this.handleChangeExperience = this.handleChangeExperience.bind(this);
        this.handleChangeTravelPref = this.handleChangeTravelPref.bind(this);
        this.handleChangeOverview = this.handleChangeOverview.bind(this);
        this.handleChangeResponsibilities = this.handleChangeResponsibilities.bind(this);
        this.handleOnSubmit = this.handleOnSubmit.bind(this);
    }

    handleChangeTitle(value) {
        this.setState(Object.assign({}, this.state, {title: value}));
    }

    handleChangeLocation(value) {
        this.setState(Object.assign({}, this.state, {location: value}));
    }

    handleChangeStartDate(value) {
        var split = value.split("/");
        this.setState(Object.assign({}, this.state, {startDate: split[2]+'-'+split[0]+'-'+split[1]}));
    }
    handleChangeEndDate(value) {
        var split = value.split("/");
        this.setState(Object.assign({}, this.state, {endDate: split[2]+'-'+split[0]+'-'+split[1]}));
    }

    handleChangeTeamSetup(value) {
        this.setState(Object.assign({}, this.state, {teamSetup: value}));
    }

    handleChangeExperience(value) {
        this.setState(Object.assign({}, this.state, {experience: value}));
    }

    handleChangeTravelPref(value) {
        this.setState(Object.assign({}, this.state, {travelling: value}));
    }
    handleChangeOverview(value) {
        this.setState(Object.assign({}, this.state, {overview: value}));
    }
    handleChangeResponsibilities(value) {
        this.setState(Object.assign({}, this.state, {responsibilities: value}));
    }

    handleOnSubmit(event)
    {
        event.preventDefault();

        let project = {
            title: this.state.title,
            company: {
                id: this.state.user.id
            },
            timeFrame: {
                startDate: this.state.startDate.length>0 ? new Date(this.state.startDate) : new Date(),
                endDate: this.state.endDate.length>0 ? new Date(this.state.endDate) : new Date("2099-01-01"),
            },
            location: this.state.location,
            teamSetup: this.state.teamSetup,
            experience: this.state.experience,
            travelling: this.state.travelling,
            overview: this.state.overview,
            responsibilities: this.state.responsibilities
        };
        this.props.onProjectSubmit(project);
    }

    render() {
        return (
            <Page isStickyFooter={true}>
                <h1 className="text-center" style={{paddingTop:"1em"}}>Add New Project</h1>
                <h4 className="text-center">You are 1 step away from being matched with Retired Professionals</h4>
                <form className="form-row" onSubmit={this.handleOnSubmit} onReset={() => this.props.history.goBack()}>
                    <div className="col-md-6" style={{marginTop:'2em'}}>
                        <h3 className="text-left">Project Overview</h3>
                        <TextField
                            label="Title"
                            id="title"
                            type="text"
                            className="md-row"
                            required={true}
                            value={this.state.title}
                            onChange={this.handleChangeTitle}
                            errorText="The project title is required"/>
                        <TextField
                            label="What is the Team Setup"
                            id="teamSetup"
                            type="text"
                            className="md-row"
                            required={true}
                            value={this.state.teamSetup}
                            onChange={this.handleChangeTeamSetup}
                            errorText="The Team Setup is required"/>
                        <TextField
                            label="Give some Project Overview"
                            id="overview"
                            type="text"
                            className="md-row customTextarea"
                            rows={3}
                            required={true}
                            value={this.state.overview}
                            onChange={this.handleChangeOverview}
                            errorText="The Project Overview is required"/>
                        <h4 className="text-left">Start Date:</h4>
                        <DatePicker  onChange={this.handleChangeStartDate} value={this.state.startDate} autoOk={true} required={true}/>
                        <h4 className="text-left">End Date:</h4>
                        <DatePicker  onChange={this.handleChangeEndDate} value={this.state.endDate} autoOk={true} required={true}/>
                    </div>
                    <div className="col-md-6"  style={{marginTop:'2em'}}>
                        <h3 className="text-left">Project Requirements</h3>
                        <TextField
                            label="What kind of Experience is Required"
                            id="experience"
                            type=" text"
                            className="md-row"
                            required={true}
                            value={this.state.experience}
                            onChange={this.handleChangeExperience}
                            errorText="The project experience is required"/>
                        <TextField
                            label="What sort of travelling would be required"
                            id="travelling"
                            type=" text"
                            className="md-row"
                            required={true}
                            value={this.state.travelling}
                            onChange={this.handleChangeTravelPref}
                            errorText="The Travelling field is required"/>
                        <TextField
                            label="Give detailed responsibilities required for this project"
                            id="responsibilities"
                            type=" text"
                            className="md-row"
                            rows={3}
                            required={true}
                            value={this.state.responsibilities}
                            onChange={this.handleChangeResponsibilities}
                            errorText="The Project Responsibilities is required"/>
                        <TextField
                            label="What is the Project Location"
                            id="location"
                            type="text"
                            className="md-row"
                            required={true}
                            value={this.state.location}
                            onChange={this.handleChangeLocation}
                            errorText="The project location is required"/>
                    </div>
                    <div className="col-md-3"/>
                    <div className="col-md-3 col-centered" style={{marginTop:'2em'}}>
                        <Button id="submit" type="submit"
                                disabled={this.state.title === undefined || this.state.title === '' || this.state.location === undefined || this.state.location === ''
                                || this.state.teamSetup === undefined || this.state.teamSetup === '' || this.state.experience === undefined || this.state.experience === ''
                                || this.state.overview === undefined || this.state.overview === '' || this.state.responsibilities === undefined || this.state.responsibilities === ''
                                || this.state.travelling === undefined || this.state.travelling === '' || (new Date(this.state.from)).getTime() >= (new Date(this.state.to)).getTime()}
                                raised primary className="md-cell md-cell--2">Submit</Button>
                        <Button id="reset" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
                        <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                    </div>
                    <div className="col-md-3"/>
                </form>
            </Page>
        );
    }
};

export default withRouter(NewProject);
