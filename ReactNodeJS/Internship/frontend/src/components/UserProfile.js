"use strict";

import React from 'react';
import {Card, Button, TextField, Subheader, SelectField, Slider, DialogContainer} from 'react-md';
import { withRouter } from 'react-router-dom';

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import DatePicker from "react-md/lib/Pickers/DatePickerContainer";
import axios from 'axios';



class UserProfile extends React.Component
{
    constructor(props) {
        super(props);
        var profileData = this.props.user._source.profileData;
        var isInterest = (profileData.interest ? true : false);
        var isExperience = (profileData.experience && profileData.experience.length>0 ? true : false);
        this.state = {
            title: (isExperience && profileData.experience[0].title ? profileData.experience[0].title : ''),
            description: (isExperience && profileData.experience[0].description ? profileData.experience[0].description : ''),
            compName: (isExperience && profileData.experience[0].compName ? profileData.experience[0].compName : ''),
            from: (isExperience && profileData.experience[0].from ? profileData.experience[0].from.substring(0,10) : ''),
            to: (isExperience && profileData.experience[0].to ? profileData.experience[0].to.substring(0,10) : ''),
            project: (isInterest && profileData.interest.projects ? profileData.interest.projects : ''),
            industries: (isInterest && profileData.interest.industries ? profileData.interest.industries: ''),
            travelPref: (profileData.travelPref ? profileData.travelPref : ''),
            industry: (profileData.matchPref ? profileData.matchPref.industry : 0),
            skillset: (profileData.matchPref ? profileData.matchPref.skillset : 0),
            interest: (profileData.matchPref ? profileData.matchPref.interest : 0),
            cv: '',
            coverLetter: '',
            photo: profileData.photo,
            photoName:'',
            selectedFile: undefined
        };

        this.photoName = null;
        this.handleChangeTitle = this.handleChangeTitle.bind(this);
        this.handleChangeDescription = this.handleChangeDescription.bind(this);
        this.handleChangeCompName = this.handleChangeCompName.bind(this);
        this.handleChangeFromDate = this.handleChangeFromDate.bind(this);
        this.handleChangeToDate = this.handleChangeToDate.bind(this);
        this.handleChangeProject = this.handleChangeProject.bind(this);
        this.handleChangeIndustries = this.handleChangeIndustries.bind(this);
        this.handleChangeTravelPref = this.handleChangeTravelPref.bind(this);
        this.handleChangeIndustry = this.handleChangeIndustry.bind(this);
        this.handleChangeSkillset = this.handleChangeSkillset.bind(this);
        this.handleChangeInterest = this.handleChangeInterest.bind(this);
        this.handleChangeCv = this.handleChangeCv.bind(this);
        this.handleChangeCoverLetter = this.handleChangeCoverLetter.bind(this);
        this.handleOnSubmit = this.handleOnSubmit.bind(this);
        this.hideModal = this.hideModal.bind(this);
        this.handleChangePhoto = this.handleChangePhoto.bind(this);
    }

    handleChangeTitle(value) {
        this.setState(Object.assign({}, this.state, {title: value}));
    }

    handleChangeDescription(value) {
        this.setState(Object.assign({}, this.state, {description: value}));
    }
    handleChangeCompName(value) {
        this.setState(Object.assign({}, this.state, {compName: value}));
    }
    handleChangeFromDate(value) {
        var split = value.split("/");
        this.setState(Object.assign({}, this.state, {from: split[2]+'-'+split[0]+'-'+split[1]}));
    }
    handleChangeToDate(value) {
        var split = value.split("/");
        this.setState(Object.assign({}, this.state, {to: split[2]+'-'+split[0]+'-'+split[1]}));
    }
    handleChangeProject(value) {
        this.setState(Object.assign({}, this.state, {project: value}));
    }
    handleChangeIndustries(value) {
        this.setState(Object.assign({}, this.state, {industries: value}));
    }
    handleChangeTravelPref(value) {
        this.setState(Object.assign({}, this.state, {travelPref: value}));
    }handleChangeIndustry(value) {
        this.setState(Object.assign({}, this.state, {industry: value}));
    }handleChangeSkillset(value) {
        this.setState(Object.assign({}, this.state, {skillset: value}));
    }handleChangeInterest(value) {
        this.setState(Object.assign({}, this.state, {interest: value}));
    }handleChangeCv(value) {
        this.setState(Object.assign({}, this.state, {cv: value}));
    }handleChangeCoverLetter(value) {
        this.setState(Object.assign({}, this.state, {coverLetter: value}));
    }
    handleChangePhoto(event) {
        this.setState(Object.assign({}, this.state, {selectedFile: event.target.files[0]}));
    };


    handleOnSubmit(event){

        event.preventDefault();

        let user = {};
        if (this.state.selectedFile != undefined) {
            const formData = new FormData();
            formData.append('profileImage', this.state.selectedFile);
            axios.post('http://localhost:3000/image', formData).then(
                response => {
                    this.photoName = response.data.filename;
                    //this.photoName.push(response.data.filename);
                    this.setState(Object.assign({}, this.state, {photoName: this.photoName}));
                    user.id = this.props.user._id;
                    user.profileData = {
                        experience:
                            [{
                                title: this.state.title,
                                description: this.state.description,
                                compName: this.state.compName,
                                from: this.state.from.length>0 ? new Date(this.state.from) : new Date("1970-01-01"),
                                to: this.state.to.length>0 ? new Date(this.state.to) : undefined
                            }],
                        matchPref: {
                            industry: this.state.industry,
                            skillset: this.state.skillset,
                            interest: this.state.interest
                        },
                        interest: {
                            projects: this.state.project,
                            industries: this.state.industries
                        },
                        travelPref: this.state.travelPref,
                        photo : this.state.photoName
                    };
                    //user.travelPref = this.state.travelPref;
                    //user.cv = this.state.cv;
                    //user.coverLetter = this.state.compName;

                    this.props.onSubmit(user);
                }
            )
        }else{
            user.id = this.props.user._id;
            user.profileData = {
                experience:
                    [{
                        title: this.state.title,
                        description: this.state.description,
                        compName: this.state.compName,
                        from: this.state.from.length>0 ? new Date(this.state.from) : new Date("1970-01-01"),
                        to: this.state.to.length>0 ? new Date(this.state.to) : undefined
                    }],
                matchPref: {
                    industry: this.state.industry,
                    skillset: this.state.skillset,
                    interest: this.state.interest
                },
                interest: {
                    projects: this.state.project,
                    industries: this.state.industries
                },
                travelPref: this.state.travelPref,
                photo : this.state.photoName
            };
            //user.travelPref = this.state.travelPref;
            //user.cv = this.state.cv;
            //user.coverLetter = this.state.compName;

            this.props.onSubmit(user);
        }
    }

    hideModal() {
        this.props.history.push('/');
    };


    render() {
        const ITEMS_WITH_ELEMENTS = [
            <Subheader key="subheader-1" primaryText="Travel" className="md-divider-border md-divider-border--bottom" />,
            'Yes', 'No', 'Maybe'
        ];

        const visible = this.props.didSubmit;
        const actions = [{
            onClick: this.hideModal,
            primary: true,
            children: 'Go to Homepage',
        }];

        return (
            <Page isStickyFooter={true}>
                <DialogContainer
                    id="success-modal"
                    visible={visible}
                    title="Signup to Wayer Successful"
                    aria-describedby="success-modal-div"
                    modal
                    actions={actions}
                    height={500}
                    width={750}
                >
                    <div id="success-modal-div" style={{padding: 50+'px'}}>
                        <h1 className="text-center md-color--secondary-text">Thank you very much.</h1>
                        <h2 className="text-center" style={{padding: 25+'px'}}>We are now searching your best project matches.</h2>
                        <h2 className="text-center" style={{padding: 25+'px'}}>The project owners will contact you by E-mail soon.</h2>
                    </div>
                </DialogContainer>

                <h1 className="text-center">My Profile</h1>
                <h4 className="text-center">You are 1 step away to be matched with interesting projects</h4>
                    <form className="form-row" onSubmit={this.handleOnSubmit} onReset={() => this.props.history.goBack()}>
                        <div className="col-md-6" style={{marginTop:'2em'}}>
                            <h3 className="text-left">Previous Experience</h3>
                            <TextField
                                label="Title"
                                id="title"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.title}
                                onChange={this.handleChangeTitle}
                                errorText="The experience title required"/>
                            <TextField
                                label="Company Name"
                                id="compName"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.compName}
                                onChange={this.handleChangeCompName}
                                errorText="The company name is required"/>
                            <TextField
                                label="Description"
                                id="description"
                                type="text"
                                className="md-row"
                                rows={2}
                                required={true}
                                value={this.state.description}
                                onChange={this.handleChangeDescription}
                                errorText="The experience description required"/>
                            <h4 className="text-left">From:</h4>
                            <DatePicker  onChange={this.handleChangeFromDate} value={this.state.from} autoOk={true} required={true}/>
                            <h4 className="text-left">To:</h4>
                            <DatePicker  onChange={this.handleChangeToDate} value={this.state.to} autoOk={true} required={true}/>
                            <h4 className="text-left" style={{marginTop:20+'px'}}>Upload your Profile image:</h4>
                            <label htmlFor="file">Upload a photo*:</label>
                            <input type="file" required={false} onChange={this.handleChangePhoto}/>


                        </div>
                        <div className="col-md-6"  style={{marginTop:'2em'}}>
                            <h3 className="text-left">Interests</h3>
                            <TextField
                                label="What kind of projects would be interesting for you"
                                id="projects"
                                type=" text"
                                className="md-row"
                                required={true}
                                value={this.state.project}
                                onChange={this.handleChangeProject}
                                errorText="The project is required"/>
                            <TextField
                                label="What kind of industry would be interesting for you"
                                id="industry"
                                type=" text"
                                className="md-row"
                                required={true}
                                value={this.state.industries}
                                onChange={this.handleChangeIndustries}
                                errorText="The industry is required"/>
                            <SelectField
                                id="industrySelectField"
                                label="Would  you like to travel"
                                placeholder="Select something"
                                menuItems={ITEMS_WITH_ELEMENTS}
                                className="md-cell md-cell--4"
                                sameWidth
                                value={this.state.travelPref}
                                onChange={this.handleChangeTravelPref}/>
                            <h3 className="text-left">How would you like to be matched for the future projects?</h3>
                            <Slider id="discrete-default-value-slider" label="Interests" discrete min={1} max={10} value={this.state.interest} onChange={this.handleChangeInterest}/>
                            <Slider id="discrete-default-value-slider" label="Skillset" discrete min={1} max={10} value={this.state.skillset} onChange={this.handleChangeSkillset}/>
                            <Slider id="discrete-default-value-slider" label="Industry" discrete min={1} max={10} value={this.state.industry} onChange={this.handleChangeIndustry}/>
                        </div>
                        <div className="col-md-3"/>
                        <div className="col-md-3 col-centered" style={{marginTop:'2em'}}>
                            <Button id="submit" type="submit"
                                    disabled={this.state.title === undefined || this.state.title === '' || this.state.compName === undefined || this.state.compName === ''
                                    || this.state.description === undefined || this.state.description === '' || this.state.project === undefined || this.state.project === ''
                                    || this.state.industries === undefined || this.state.industries === '' || (new Date(this.state.from)).getTime()>=(new Date(this.state.to)).getTime() ? true : false}
                                    raised primary className="md-cell md-cell--2">Confirm</Button>
                            <Button id="reset" type="reset" raised secondary className="md-cell md-cell--2">Dismiss</Button>
                            <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                        </div>
                        <div className="col-md-3"/>
                    </form>
            </Page>
        );
    }
};

export default withRouter(UserProfile);
