"use strict";

import React from 'react';
import { Card, Button, TextField, Checkbox, Grid, Cell, Subheader, SelectField } from 'react-md';
import { withRouter, Link } from 'react-router-dom';
import ReactQuill from 'react-quill';
import 'react-quill/dist/quill.snow.css';

import Page from './Page';
import 'react-md/dist/react-md.indigo-pink.min.css'

class UserContact extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            message: '',
            copyCheckbox: false,
            privacyCheckbox: false,
            projectRadio: ''
        };

        this.handleChangeMessage = this.handleChangeMessage.bind(this);
        this.handleChangecopyCheckbox = this.handleChangecopyCheckbox.bind(this);
        this.handleChangeprivacyCheckbox = this.handleChangeprivacyCheckbox.bind(this);
        this.handleChangeprojectRadio = this.handleChangeprojectRadio.bind(this);

        this.handleMessageSubmit = this.handleMessageSubmit.bind(this);
    }

    handleChangeMessage(value) {
        this.setState(Object.assign({}, this.state, {message: value}));
    }

    handleChangecopyCheckbox(value) {
        this.setState(Object.assign({}, this.state, {copyCheckbox: value}));
    }

    handleChangeprivacyCheckbox(value) {
        this.setState(Object.assign({}, this.state, {privacyCheckbox: value}));
    }

    handleChangeprojectRadio(e) {
        if(e.target.checked) {
            this.setState(Object.assign({}, this.state, {projectRadio: e.target.value}));
        }
    }

    handleMessageSubmit(event) {
        event.preventDefault();

        let contactData = {
            message: this.state.message,
            address: this.props.user._source.username,
            shouldCopy: this.state.copyCheckbox,
            projectId: this.state.projectRadio.length===0 ? this.props.projects[0]._id : this.state.projectRadio
        };

        this.props.onMessageSubmit(contactData);
    }

    render() {
        const ResultsGrid = this.props.projects.map((rp,index) =>
            <div className="col-md-4">
                <Card className="userContactProjectBox" style={{padding: 20+'px'}}>
                    <input onChange={this.handleChangeprojectRadio} type="radio" value={rp._id} name="projectRadio" checked={this.state.projectRadio===rp._id || index===0}/>
                    <p className="userContact1stBoxText">{rp._source.title}</p>
                </Card>
            </div>
        );
        return (
            <Page>
                <div className="col-md-12">
                    <div className="col-md-2"/>
                    <div className="col-md-8 userContact1stBox">
                        <h2>{this.props.user._source.profileData.firstName+" "+this.props.user._source.profileData.lastName}</h2>
                        <h3>{this.props.user._source.profileData.experience[0].title+" in"}</h3>
                        <h3>{this.props.user._source.profileData.experience[0].compName}</h3>
                    </div>
                    <div className="col-md-2"/>
                </div>
                <form className="md-grid" onSubmit={this.handleMessageSubmit} onReset={() => this.props.history.goBack()}>
                    <div className="col-md-12">
                        <div className="col-md-2"/>
                        <div className="col-md-8">
                            <h2>Choose the Projects...</h2>
                            {ResultsGrid}
                        </div>
                        <div className="col-md-2"/>
                    </div>
                    <div className="col-md-12" style={{marginTop: 50+'px'}}>
                        <div className="col-md-2"/>
                        <div className="col-md-8">
                            <h2 style={{marginBottom: 20+'px'}}>Write the message to {this.props.user._source.profileData.firstName} ...</h2>
                            <ReactQuill value={this.state.message}
                                        onChange={this.handleChangeMessage} style={{height: 400+'px',paddingBottom: 50+'px'}}/>
                            {/*<TextField
                                helpText="Write your message here..."
                                helpOnFocus={true}
                                lineDirection="center"
                                maxLength={800}
                                rows={14}
                                maxRows={14}
                                id="msgField"
                                type="text"
                                className="md-row userContactTextBox"
                                required={true}
                                fullWidth
                                value={this.state.message}
                                onChange={this.handleChangeMessage}
                                errorText="Message is required"/>*/}
                            <Checkbox
                                id="copy-checkbox"
                                name="copy-checkbox"
                                label="Send me the copy of message to my email."
                                onChange={this.handleChangecopyCheckbox}
                            />
                            <Checkbox
                                id="copy-privacy"
                                name="copy-privacy"
                                label="I have read and accepted the privacy policy."
                                onChange={this.handleChangeprivacyCheckbox}
                            />
                        </div>
                        <div className="col-md-2"/>
                    </div>
                    <div className="col-md-12" style={{paddingTop: 50+'px', paddingBottom: 50+'px'}}>
                        <div className="col-md-4"/>
                        <div className="col-md-2">
                            <Button id="resetSignup" type="reset" raised secondary className="userContactBackBtn">Back</Button>
                        </div>
                        <div className="col-md-2">
                            <Button id="submit" type="submit" disabled={this.state.message == undefined || this.state.message == '' || !this.state.privacyCheckbox ? true : false} raised primary className="userContactSendBtn">Send</Button>
                        </div>
                        <div className="col-md-4"/>
                    </div>
                </form>
            </Page>
        );
    }
};

export default withRouter(UserContact);
